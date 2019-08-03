//
// hyd_unit.c
// 2009.12.31, 2018.07.23 monday
//---------------------------------------------------------
// speed setting
//     codeSpeed1 when exSensRef > ( codePessSensRef - 0.1 )
//     codeSpeed2 when exSensRef < codePessSensRef
//
#include	<header.h>
#include	<extern.h>

void hyd_unit_proc(int state, double * ref_out)
{
	static int hyd_low_count=0;
	static double ref_in = 1.0;
	static unsigned long ulCount;
	double exSens;

 	if( ulGetNow_mSec( )== ulCount ) return;
	ulCount = ulGetNow_mSec( );

	exSens = adcExSensor * 0.0002441406250;

	if( state == STATE_RUN ){
		if( exSens < ( codePresSensRef - 0.1 ) ) hyd_low_count++;
		else if ( exSens >  codePresSensRef ) hyd_low_count --;

		if( hyd_low_count > 3){
		    hyd_low_count = 4; ref_in = 0.9 ; // ref_in = codeSpeed2;    // hyd_max_ref
		} else if( hyd_low_count <= 0 ){
			hyd_low_count = 0 ; ref_in = 0.2 ; // codeSpeed1;    // hyd_low_ref
		}
	}
	else	ref_in = 0.0;
	RefFunc( ref_in, ref_out );
}

#define btn_start_ref       0.01

int hyd_unit_loop_proc()
{
	int LoopCtrl;
	int iTripCode=0;

	int cmd;
	double fReference;
	double dtemp;

    commonVariableInit();
	iTripCode = HardwareParameterVerification();
	if( iTripCode !=0 ) return iTripCode;

	iTripCode = SL_SPEED_CNTL_Parameter();        	// csk_debug
	if( iTripCode != 0)	return iTripCode;			// csk_debug

	IER &= ~M_INT3;      // debug for PWM
	InitEPwm_ACIM_Inverter(); 	// debug
	EPwm1Regs.ETSEL.bit.INTEN = 1;    		            // Enable INT
	IER |= M_INT3;      // debug for PWM

	gRunFlag =1;
	gfRunTime = 0.0; 
	LoopCtrl = 1;		

	reference_in = btn_start_ref;
	gMachineState = STATE_INIT_RUN;    strncpy(MonitorMsg,"INIT",20);
 	while(LoopCtrl == 1)
	{
		if(gPWMTripCode != 0){
			iTripCode = gPWMTripCode;
			LoopCtrl = 0;
			break;
		}		
		//get_command(& cmd ,&fReference);	// Command�� �Է� ����
        digital_input_proc( & cmd, & dtemp);
		monitor_proc();

		if( cmd == CMD_START) reference_in = 0.5;
		else if( cmd == CMD_STOP) reference_in = 0.0;

		switch( gMachineState )
		{
        case STATE_INIT_RUN:
            if( cmd == CMD_STOP){
                strncpy(MonitorMsg,"READY",20); gMachineState = STATE_READY; LoopCtrl= 0;
            } else if( gfRunTime < 0.2 ){
                Freq_ref=0.0;   rpm_ref=0.0; reference_out = 0.0;
            } else{
                strncpy(MonitorMsg,"RUN",20); gMachineState = STATE_RUN; reference_out = MIN_REF;
            }
            break;
		case STATE_RUN:
		    if(  cmd == CMD_STOP ){
				strncpy(MonitorMsg," INV GO STOP        ",20);
				reference_in = 0.0; gMachineState = STATE_GO_STOP;
		    } else if( gfRunTime < 10.0 ){
		         reference_in = 0.8;
                rampFunc1(reference_in, & reference_out,3.0);
		    } else{
                hyd_unit_proc( gMachineState, & reference_out);
			}

		case STATE_GO_STOP:
			if( cmd == CMD_START ) {
									  //"01234567890123456789"	
				strncpy(MonitorMsg," INVERTER RUN       ",20);
				gMachineState = STATE_RUN;
				hyd_unit_proc( gMachineState, & reference_out);
			}				
			else if( fabs( reference_out ) < 0.005 ){
				strncpy(MonitorMsg," INVERTER READY     ",20);
				gMachineState = STATE_READY;
				LoopCtrl =0;
			}
			else{		// debug
				hyd_unit_proc(  gMachineState, & reference_out);
			}
			break;
		}
	}
	return iTripCode;
}

void hyd_unit_control()
{
	double 	IntegralLimit;
	
	wr_ref=wr_rat * reference_out;
	
	wr_CycleIndex++;
	wr_Cycle+=Ts;
	if (wr_CycleIndex>=wr_CntlPeriodIndex)
	{
		wr_err=wr_ref-wr;
		wr_ErrInt+=Ki_wr*wr_err*inv_P_pair*wr_Cycle;
		IntegralLimit=Kp_wr*fabs(K_Damp_wr*wr_ref-wr)*inv_P_pair + Is_DQ_max[QS];
		if      (wr_ErrInt>IntegralLimit)   wr_ErrInt=IntegralLimit;
		else if (wr_ErrInt<-IntegralLimit)  wr_ErrInt=-IntegralLimit;

		Is_DQ_ref[QS]=Kp_wr*(K_Damp_wr*wr_ref-wr)*inv_P_pair+wr_ErrInt;

		if (gfRunTime<0.05)		wr_ErrInt=0.0;
		
		wr_Cycle=0.0;
		wr_CycleIndex=0;
	}		
	SL_VectCntl_SFRF();	
}

void hyd_unit_torq_ctrl()
{
	Te_ref=Te_rat*reference_out;
	inv_Fs_ref=1.0/Fs_ref;
	Is_DQ_ref[QS]=inv_Kt*Te_ref*inv_Fs_ref;
	SL_VectCntl_SFRF();
}

//-----------------------
// end of hyd_unit.c
//------------------------

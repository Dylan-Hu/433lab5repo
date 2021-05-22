
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
*
* Modified for lab 5 use. The result of the filtering can be found using a pair of headphones,
* with the output of the filtered signal being in one ear while the original signal is played
* through the other
* 5/21/2021 Dylan Huntsman
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/******************************************************************************************
 * Module Defines
 *****************************************************************************************/
#define SINESIZE            DSP_SAMPLES_PER_BLOCK-NUM_TAPS+1
#define SAMPLE_RATE_HZ      48000
#define FREQ_NORM(x)        x*SAMPLE_RATE_HZ/DSP_SAMPLES_PER_BLOCK
#define coef                (sizeof(iirCoeffF32)/sizeof(iirCoeffF32[0]))
/******************************************************************************************
 * Private Variables
 *****************************************************************************************/
//Filter Coefficients created using Matlab Filter Builder app

static float32_t iirCoeffF32[] = {0.0149402948728242,0.0298805897456485,0.0149402948728242,1.64567845316408,-0.884100194705574, //"good"
                                    0.0346509256406515,0.069301851281303,0.0346509256406515,1.50923323688472,-0.727887139801492,
                                    0.04978343575405,0.0995668715081001,0.04978343575405,1.4008871422837,-0.603844136418369,
                                    0.0609293483241184,0.121858696648237,0.0609293483241184,1.31701996867219,-0.507826498326883,
                                    0.0685141128971049,0.13702822579421,0.0685141128971049,1.25455281416347,-0.436309260104376,
                                    0.0726692607067334,0.145338521413467,0.0726692607067334,1.21108468517865,-0.386543578280985,
                                    0.0723690457105563,0.144738091421113,0.0723690457105563,1.18490907636934,-0.356575713319669,
                                    0.0507384592951268,0.101476918590254,0.0507384592951268,1.17498931525589,-0.345218802247917,
                                    0.0426648472117412,0.0853296944234825,0.0426648472117412,1.17795666301146,-0.348616051858425,
                                    0.0433153028769804,0.0866306057539608,0.0433153028769804,1.19591544254393,-0.369176654051849,
                                    0.0445703456797687,0.0891406913595374,0.0445703456797687,1.23056659281225,-0.408847975531323,
                                    0.0464795149422296,0.0929590298844593,0.0464795149422296,1.2832778715465,-0.469195931315423,
                                    0.0491171323017833,0.0982342646035666,0.0491171323017833,1.35610126471937,-0.552569793926505,
                                    0.052582693366507,0.105165386733014,0.052582693366507,1.45178379996916,-0.662114573435192,
                                    0.0569985788669089,0.113997157733818,0.0569985788669089,1.57370435256082,-0.801698668028451,
                                    0.0317483888649152,0.0634967777298303,0.0317483888649152,1.72562693832679,-0.975631414654318,
                                    0.0203650670184292,0.0407301340368585,0.0203650670184292,1.60872350866429,-0.841791311099836,
                                    0.0392915281430391,0.0785830562860783,0.0392915281430391,1.47966060758751,-0.694030102596136,
                                    0.0542004478302453,0.108400895660491,0.0542004478302453,1.3777566566866,-0.577362564436103,
                                    0.0656041056304615,0.131208211260923,0.0656041056304615,1.29950268312461,-0.487771352652799,
                                    0.0739280958737497,0.147856191747499,0.0739280958737497,1.24198302261988,-0.421918388880934,
                                    0.0795008170903302,0.15900163418066,0.0795008170903302,1.20297131143578,-0.377254759423773,
                                    0.0723837881813297,0.144767576362659,0.0723837881813297,1.18093136130069,-0.352021716929362,
                                    0.0425931673427951,0.0851863346855902,0.0425931673427951,1.1759776150423,-0.346350284413482,
                                    0.0430974148325792,0.0861948296651583,0.0430974148325792,1.18989965459514,-0.362289313925461,
                                    0.0441976113678274,0.0883952227356548,0.0441976113678274,1.22027557116379,-0.397066016635098,
                                    0.0459372660613812,0.0918745321227623,0.0459372660613812,1.26830663119409,-0.452055695439617,
                                    0.0483844184725087,0.0967688369450174,0.0483844184725087,1.33587137539172,-0.529409049281755,
                                    0.0516323317496895,0.103264663499379,0.0516323317496895,1.42554475607327,-0.632074083072028,
                                    0.0557981503319964,0.111596300663993,0.0557981503319964,1.54056107692335,-0.763753678251336,
                                    0.0610161502142276,0.122032300428455,0.0610161502142276,1.68462763594239,-0.928692236799303};

static float32_t iirCoeffPt4F32[] = {1,2,1,1.72562693832679,-0.975631414654318,         //"Bad"
                                    1,2,1,1.68462763594239,-0.928692236799303,
                                    1,2,1,1.64567845316408,-0.884100194705574,
                                    1,2,1,1.60872350866429,-0.841791311099836,
                                    1,2,1,1.57370435256082,-0.801698668028451,
                                    1,2,1,1.54056107692335,-0.763753678251336,
                                    1,2,1,1.50923323688472,-0.727887139801492,
                                    1,2,1,1.47966060758751,-0.694030102596136,
                                    1,2,1,1.45178379996916,-0.662114573435192,
                                    1,2,1,1.42554475607327,-0.632074083072028,
                                    1,2,1,1.4008871422837,-0.603844136418369,
                                    1,2,1,1.3777566566866,-0.577362564436103,
                                    1,2,1,1.35610126471937,-0.552569793926506,
                                    1,2,1,1.33587137539172,-0.529409049281755,
                                    1,2,1,1.31701996867219,-0.507826498326883,
                                    1,2,1,1.29950268312461,-0.487771352652799,
                                    1,2,1,1.2832778715465,-0.469195931315423,
                                    1,2,1,1.26830663119409,-0.452055695439617,
                                    1,2,1,1.25455281416347,-0.436309260104376,
                                    1,2,1,1.24198302261988,-0.421918388880934,
                                    1,2,1,1.23056659281225,-0.408847975531323,
                                    1,2,1,1.22027557116379,-0.397066016635099,
                                    1,2,1,1.21108468517865,-0.386543578280985,
                                    1,2,1,1.20297131143578,-0.377254759423774,
                                    1,2,1,1.19591544254393,-0.369176654051849,
                                    1,2,1,1.18989965459514,-0.362289313925461,
                                    1,2,1,1.18490907636934,-0.356575713319669,
                                    1,2,1,1.18093136130069,-0.352021716929362,
                                    1,2,1,1.17795666301146,-0.348616051858425,
                                    1,2,1,1.1759776150423,-0.346350284413482,
                                    1,2,1,1.17498931525589,-0.345218802247917};

static float32_t iirCoeffDivF32[] = {0.00373507371820606,0.00747014743641212,0.00373507371820606,0.41141961329102,-0.221025048676394, //Processed for Q31
                                    0.00866273141016288,0.0173254628203258,0.00866273141016288,0.37730830922118,-0.181971784950373,
                                    0.0124458589385125,0.024891717877025,0.0124458589385125,0.350221785570926,-0.150961034104592,
                                    0.0152323370810296,0.0304646741620592,0.0152323370810296,0.329254992168046,-0.126956624581721,
                                    0.0171285282242762,0.0342570564485525,0.0171285282242762,0.313638203540866,-0.109077315026094,
                                    0.0181673151766833,0.0363346303533667,0.0181673151766833,0.302771171294663,-0.0966358945702462,
                                    0.0180922614276391,0.0361845228552781,0.0180922614276391,0.296227269092334,-0.0891439283299173,
                                    0.0126846148237817,0.0253692296475634,0.0126846148237817,0.293747328813972,-0.0863047005619792,
                                    0.0106662118029353,0.0213324236058706,0.0106662118029353,0.294489165752865,-0.0871540129646063,
                                    0.0108288257192451,0.0216576514384902,0.0108288257192451,0.298978860635982,-0.0922941635129623,
                                    0.0111425864199422,0.0222851728398843,0.0111425864199422,0.307641648203062,-0.102211993882831,
                                    0.0116198787355574,0.0232397574711148,0.0116198787355574,0.320819467886626,-0.117298982828856,
                                    0.0122792830754458,0.0245585661508917,0.0122792830754458,0.339025316179843,-0.138142448481626,
                                    0.0131456733416268,0.0262913466832535,0.0131456733416268,0.362945949992291,-0.165528643358798,
                                    0.0142496447167272,0.0284992894334544,0.0142496447167272,0.393426088140204,-0.200424667007113,
                                    0.00793709721622879,0.0158741944324576,0.00793709721622879,0.431406734581696,-0.24390785366358,
                                    0.00509126675460731,0.0101825335092146,0.00509126675460731,0.402180877166072,-0.210447827774959,
                                    0.00982288203575979,0.0196457640715196,0.00982288203575979,0.369915151896877,-0.173507525649034,
                                    0.0135501119575613,0.0271002239151227,0.0135501119575613,0.344439164171651,-0.144340641109026,
                                    0.0164010264076154,0.0328020528152307,0.0164010264076154,0.324875670781152,-0.1219428381632,
                                    0.0184820239684374,0.0369640479368749,0.0184820239684374,0.31049575565497,-0.105479597220234,
                                    0.0198752042725825,0.0397504085451651,0.0198752042725825,0.300742827858946,-0.0943136898559434,
                                    0.0180959470453324,0.0361918940906649,0.0180959470453324,0.295232840325173,-0.0880054292323405,
                                    0.0106482918356988,0.0212965836713976,0.0106482918356988,0.293994403760575,-0.0865875711033706,
                                    0.0107743537081448,0.0215487074162896,0.0107743537081448,0.297474913648786,-0.0905723284813653,
                                    0.0110494028419569,0.0220988056839137,0.0110494028419569,0.305068892790947,-0.0992665041587746,
                                    0.0114843165153453,0.0229686330306906,0.0114843165153453,0.317076657798523,-0.113013923859904,
                                    0.0120961046181272,0.0241922092362543,0.0120961046181272,0.33396784384793,-0.132352262320439,
                                    0.0129080829374224,0.0258161658748448,0.0129080829374224,0.356386189018317,-0.158018520768007,
                                    0.0139495375829991,0.0278990751659982,0.0139495375829991,0.385140269230838,-0.190938419562834,
                                    0.0152540375535569,0.0305080751071138,0.0152540375535569,0.421156908985597,-0.232173059199826};

static q31_t iirCoeffQ31[coef];
static q31_t iirCoeffBQ31[coef];
static  arm_biquad_casd_df1_inst_f32  iirF32;
static arm_biquad_casd_df1_inst_q31 iirbadQ31;
static arm_biquad_casd_df1_inst_q31 iirgoodQ31;
static float32_t p0State[coef];
static q31_t badStateQ31[coef];
static q31_t goodStateQ31[coef];

static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];

static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};

//Single struct to hold all message info
typedef struct{

    q31_t mixedQ31[DSP_SAMPLES_PER_BLOCK];    //part 4 unfiltered message
    q31_t badfiltQ31[DSP_SAMPLES_PER_BLOCK];
    q31_t goodfiltQ31[DSP_SAMPLES_PER_BLOCK];
    float32_t mixedF32[DSP_SAMPLES_PER_BLOCK];  //Holds sum of tones.
    float32_t filtF32[DSP_SAMPLES_PER_BLOCK];
    DSP_BLOCK_T inputQ;                     //variable to pass original message to CODEC
    DSP_BLOCK_T filteredAudio;              //variable to pass filtered message to CODEC
} DSP_AUDIO;

static DSP_AUDIO signalTone;
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void SineInit(void);
static void FilterInit(void);
static void FloatFilter(void);
static void FixedFilter(void);
static void dspTask(void *p_arg);
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    SineInit();
    FilterInit();
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();
}

/*******************************************************************************************
* dspTask - Performs filtering on mixed and modulated signals once, handles audio output
*******************************************************************************************/
static void dspTask(void *p_arg){
    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){

        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        FloatFilter();
        FixedFilter();
        dspOutBuffer[DSP_LEFT_CH][buffer_index] = signalTone.filteredAudio; //Left Channel
        dspOutBuffer[DSP_RIGHT_CH][buffer_index] = signalTone.inputQ; //Right Channel
        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}
/*******************************************************************************************
* SineInit - Creates a tone with three frequency components to be filtered out.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void SineInit(void){
    float32_t f1 = 3750;
    float32_t f2 = 5625;
    float32_t f3 = 7500;
    INT16U i;
    //max value in totalF32 is +/- 0.25, no overflow.
    for(i=0;i<DSP_SAMPLES_PER_BLOCK;i++){
        signalTone.mixedF32[i]=(arm_cos_f32(2*PI*i*f1/SAMPLE_RATE_HZ)+
                arm_cos_f32(2*PI*i*f2/SAMPLE_RATE_HZ)+arm_cos_f32(2*PI*i*f3/SAMPLE_RATE_HZ))/12;
    }

    arm_float_to_q31(signalTone.mixedF32, signalTone.mixedQ31, DSP_SAMPLES_PER_BLOCK);
    arm_float_to_q31(signalTone.mixedF32, signalTone.inputQ.samples, DSP_SAMPLES_PER_BLOCK);
}
/*******************************************************************************************
* FilterInit - Creates all the necessary IIR filters for the lab.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void FilterInit(void){
    INT16U i;
    for (i = 0; i < coef-1; i++){
        p0State[i] = 0;
        badStateQ31[i] = 0;
        goodStateQ31[i] = 0;
    }
    arm_float_to_q31(iirCoeffPt4F32, iirCoeffBQ31, coef);               //Bad Fixed Point Filter
    //arm_float_to_q31(iirCoeffF32, iirCoeffBQ31, coef);               //Bad Fixed Point Filter - also doesn't work with coefficients for part 3
    arm_float_to_q31(iirCoeffDivF32, iirCoeffQ31, coef);                //Good Fixed Point Filter
    arm_biquad_cascade_df1_init_f32(&iirF32, (uint8_t)(coef/5),
            iirCoeffF32, p0State);
    arm_biquad_cascade_df1_init_q31(&iirbadQ31, (uint8_t)(coef/5),
            iirCoeffBQ31, badStateQ31, 0);
    arm_biquad_cascade_df1_init_q31(&iirgoodQ31, (uint8_t)(coef/5),
            iirCoeffQ31, goodStateQ31, 2);
}


/*******************************************************************************************
* FloatFilter - Filtering function for part 3.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void FloatFilter(void){
    arm_biquad_cascade_df1_f32(&iirF32, signalTone.mixedF32,
                signalTone.filtF32, (uint32_t) DSP_SAMPLES_PER_BLOCK);
    arm_float_to_q31(signalTone.filtF32, signalTone.filteredAudio.samples, DSP_SAMPLES_PER_BLOCK);
}
/*******************************************************************************************
* FixedFilter - Filtering function for part 4.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void FixedFilter(void){
    arm_biquad_cascade_df1_q31(&iirbadQ31, signalTone.mixedQ31,
                signalTone.badfiltQ31, (uint32_t) DSP_SAMPLES_PER_BLOCK);
    arm_biquad_cascade_df1_q31(&iirgoodQ31, signalTone.mixedQ31,
                signalTone.goodfiltQ31, (uint32_t) DSP_SAMPLES_PER_BLOCK);

}
/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){

    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];

}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){

    return dspParams.ssize;

}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){

    return dspParams.srate;

}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){

    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];

}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}



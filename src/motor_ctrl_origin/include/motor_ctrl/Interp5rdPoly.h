#include <eigen3/Eigen/Dense>
#include <cmath>


using namespace Eigen;
/************************************************************************/
/*
多项式插值待定参数计算

    功能：考虑起点、终点速度、加速度为0的简化情况，计算五次多项式插值待定参数

    输入：float q0，初始状态
          float qf，终止状态
          float tf，运动时间 (单位：s)

    输出：float a0345[4]，多项式参数，a0, a3, a4, a5
                                                                          */
/************************************************************************/
void Interp5rdPoly_Param_P2P_Zrros_fcn(float q0, float qf, float tf, float a0345[4])
{
    float tf_2, tf_3, tf_4, tf_5;

    tf_2 = tf * tf;
    tf_3 = tf_2 * tf;
    tf_4 = tf_3 * tf;
    tf_5 = tf_4 * tf;

    a0345[0] = q0;
    a0345[1] = 10 * (qf - q0) / tf_3;
    a0345[2] = -15 * (qf - q0) / tf_4;
    a0345[3] = 6 * (qf - q0) / tf_5;
}

/************************************************************************/
/*			五次多项式插值

    功能：考虑起点、终点速度、加速度为0的简化情况，给定5次多项式参数和当前时间，规划当前的关节角，角速度，角加速度

    输入：float tt，时间变量，循环变量
          float a0345[4]，多项式参数，a0, a3, a4, a5

    输出：float qq_data[3] 关节参数，包括关节角(单位：rad)，角速度(单位：rad/s)，角加速度(单位：rad/s^2)
                                                                          */
/************************************************************************/
void  Interp5rdPoly_Data_t_P2P_Zrros_fcn(float tt, float a0345[4], float qq_data[3])
{

    float a0, a3, a4, a5, qt, qvt, qat;
    float tt_2, tt_3, tt_4, tt_5;

    tt_2 = tt * tt;
    tt_3 = tt_2 * tt;
    tt_4 = tt_3 * tt;
    tt_5 = tt_4 * tt;

    a0 = a0345[0];
    a3 = a0345[1];
    a4 = a0345[2];
    a5 = a0345[3];

    qt = a0 + a3 * tt_3 + a4 * tt_4 + a5 * tt_5;
    qvt = 3 * a3 * tt_2 + 4 * a4 * tt_3 + 5 * a5 * tt_4;
    qat = 6 * a3 * tt + 12 * a4 * tt_2 + 20 * a5 * tt_3;

    qq_data[0] = qt;
    qq_data[1] = qvt;
    qq_data[2] = qat;
}

/*************************************************************************/
/*			五次多项式插值

    功能：考虑起点、终点速度、加速度为任意数值，给定5次多项式参数和当前时间，规划当前的关节角，角速度，角加速度
    输入：float tt，时间变量，循环变量
          float a_i[6]，多项式参数，a0, a1, a2, a3, a4, a5
    输出：float qq_data[3] 关节参数，包括关节角(单位：rad)，角速度(单位：rad/s)，角加速度(单位：rad/s^2)
                                                                          */
/************************************************************************/
void Interp5rdPoly_value(float q0, float qv0, float qa0, float qf, float qvf, float qaf, float tf, float a_i[6])
{
    float tf_1, tf_2, tf_3, tf_4, tf_5;
    tf_1 = tf;
    tf_2 = tf * tf;
    tf_3 = tf_2 * tf;
    tf_4 = tf_3 * tf;
    tf_5 = tf_4 * tf;
    a_i[0] = q0;
    a_i[1] = qv0;
    a_i[2] = qa0 / 2;
    a_i[3] = (20 * (qf - q0) - (8 * qvf + 12 * qv0) * tf_1 + (qaf - 3 * qa0) * tf_2) / (2 * tf_3);
    a_i[4] = (-30 * (qf - q0) + (14 * qvf + 6 * qv0) * tf_1 - (2 * qaf - 3 * qa0) * tf_2) / (2 * tf_4);
    a_i[5] = (12 * (qf - q0) - (6 * qvf + 6 * qv0) * tf_1 + (qaf - qa0) * tf_2) / (2 * tf_5);
}
void Interp5rdPoly(float tt, float a_i[6], float qq_data[3])
{
    float a0, a1, a2, a3, a4, a5, qt, qvt, qat;
    float tt_1, tt_2, tt_3, tt_4, tt_5;
    tt_1 = tt;
    tt_2 = tt * tt;
    tt_3 = tt_2 * tt;
    tt_4 = tt_3 * tt;
    tt_5 = tt_4 * tt;

    a0 = a_i[0];
    a1 = a_i[1];
    a2 = a_i[2];
    a3 = a_i[3];
    a4 = a_i[4];
    a5 = a_i[5];

    qt = a0 + a1 * tt_1 + a2 * tt_2 + a3 * tt_3 + a4 * tt_4 + a5 * tt_5;
    qvt = a1 + 2 * a2 * tt_1 + 3 * a3 * tt_2 + 4 * a4 * tt_3 + 5 * a5 * tt_4;
    qat = 2 * a2 + 6 * a3 * tt + 12 * a4 * tt_2 + 20 * a5 * tt_3;

    qq_data[0] = qt;
    qq_data[1] = qvt;
    qq_data[2] = qat;
}

/*************************************************************/

/*	函数名称:		nfPathPlanOffLine5(五次多项式)关节空间轨迹规划
    函数功能:		利用五次多项式插值的离线规划，根据星上时间、机械臂当前关节角、关节空间运动结点、节点数，计算给定时间的期望关节角度、关节角速度、关节角加速度
    参    数:
                    cstarTime：输入变量，星上时间
                    JntCurrentAngleDeg[7]：输入变量，机械臂当前关节角、度(单位：度°)
                    fJntP2PNodeDeg[MAX_POINT_NUM][8]：输入变量，前7列为关节角，关节空间运动节点(单位：度°，最后一列为时间，单位：s)
                    iNodeNum：输入变量，节点数

                    fDesiredJntAngleDeg[7]：输出变量，期望关节角度(单位：度°)
                    fDesiredJntRateDeg [7]：输出变量，期望期望关节角速度(单位：度°/s)
                    fDesiredJntAccDeg [7]：输出变量，期望期望关节加速度	(单位：度°/s^2)
    返回值:
                     PlanningSuccesFlg=1: 离线规划成功
                                      =0: 离线规划没成功
*/
/*************************************************************/
int nfPathPlanOffLine5(float cstarTime, float JntCurrentAngleDeg[7], float fJntP2PNodeDeg[][8], int iNodeNum,
                       float fDesiredJntAngleDeg[7], float fDesiredJntRateDeg[7], float fDesiredJntAccDeg[7])
{
    int i, k;
    // float dfFirstVel[7];		/*	初始速度变量	*/
    // float dfLastVel[7];		/*	终止速度变量	*/
    int bTimeSeqRight = 1;             /*	时间序列合理标志	*/
    int TmpFlag = 1;                   /*	单步合理标志	*/
    int PlanningSuccesFlg;             /*	规划成功标志	*/
    int g_bFinishFlag;                 /*规划失败标志*/
    int g_iPlanFirstCircle = 0;        /*	第一次规划标志	*/
    int g_bStartTimeIsZeros;           /* 第一次规划的时间标志*/
    int g_bTimeSeqRight;               /* 时间序列递增标志 */
    int g_iNodeNum_seq;                /* 时间序列标志位 */
    int g_bCurrentIllEquInitErrorFlag; /* 起始点规划标志位*/

    /*	处理超限问题的局部变量	*/
    float fTmpDesiredJntAngleDeg[7];
    float fTmpDesiredJntRateDeg[7];
    float fTmpDesiredJntAccDeg[7];
    float g_fPlanTimeMax;                                 // 规划最大时间
    float g_iPlanStepMax;                                 // 规划最大步长
    float g_fJntErrLimit;                                 // 允许的关节角误差；
    float g_fSampleTime_OBC_Hz = 100;                     // 控制周期
    float g_fSampleTime_OBC = 1.0 / g_fSampleTime_OBC_Hz; /* 采样时间 */
    float g_fTimeSeq[iNodeNum];                           /* 时间序列 */
    float g_fPlanRunTime_OBC;                             /* 运行时间 */
    float g_fPlanStartTime_OBC;                           /* 启动时间 */
    float g_afJntAngleBoundaryDeg[7][3];                  /*关节角边界*/
    int g_bNoFeasiblePath;                                /* 关节角超限标志位 */

    int MAX_POINT_NUM;   // 最大关节点数
    float g_a0345[7][4]; /* 五次多项式系数 */
    /* 判断规划数据点数是否满足要求，不满足返回 */
    if ((iNodeNum < 2) || (iNodeNum > MAX_POINT_NUM))
    {
        PlanningSuccesFlg = 0;
        g_bFinishFlag = 1;
        for (i = 0; i < 7; i++)
        {
            fDesiredJntAngleDeg[i] = JntCurrentAngleDeg[i];
            fDesiredJntRateDeg[i] = 0.0f;
            fDesiredJntAccDeg[i] = 0.0f;
        }
        return PlanningSuccesFlg;
    }

    /*	判断是否为第一次规划	*/
    if (g_iPlanFirstCircle == 0)
    {
        g_fPlanTimeMax = fJntP2PNodeDeg[iNodeNum - 1][7];              /*	记录规划最大时间	*/
        g_iPlanStepMax = (int)(g_fPlanTimeMax * g_fSampleTime_OBC_Hz); /*	根据时间计算需要的最大步长	*/
        /*	判断初始角度是否满足误差要求(fJntP2PNodeDeg的第一行0-6列为7个初始角度)	*/
        if ((fabs((JntCurrentAngleDeg[0] - fJntP2PNodeDeg[0][0])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[1] - fJntP2PNodeDeg[0][1])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[2] - fJntP2PNodeDeg[0][2])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[3] - fJntP2PNodeDeg[0][3])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[4] - fJntP2PNodeDeg[0][4])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[5] - fJntP2PNodeDeg[0][5])) < g_fJntErrLimit) && (fabs((JntCurrentAngleDeg[6] - fJntP2PNodeDeg[0][6])) < g_fJntErrLimit))
        {
            float g_fPlanStartTime_OBC = cstarTime; // 启动时间=0
            float g_fPlanRunTime_OBC = 0.0f;

            for (i = 0; i < iNodeNum; i++)
            {
                /*	提取时间序列（fJntP2PNodeDeg第8列为时间）	*/
                g_fTimeSeq[i] = fJntP2PNodeDeg[i][7];
            }

            /************************************************************************/
            /*						判断时间序列是否合理                            */
            /************************************************************************/
            /*	初始时刻应为0：时间序列的第一个节点所对应时间为零	*/
            if (fabs(g_fTimeSeq[0]) >= 1e-6)
            {
                g_bStartTimeIsZeros = 0;
                g_bTimeSeqRight = 0;
            }
            else
            {
                g_bStartTimeIsZeros = 1;
                g_bTimeSeqRight = 1;
                /*	判断时间序列是否递增	*/
                for (i = 0; i < iNodeNum - 1; i++)
                {
                    if (g_fTimeSeq[i] >= g_fTimeSeq[i + 1])
                    {
                        g_bTimeSeqRight = 0;
                        break;
                    }
                }
            }

            /*	如果时间起点及时间序列合理，进行规划计算	*/
            if (g_bStartTimeIsZeros && g_bTimeSeqRight)
            {

                for (k = 0; k < 7; k++)
                {
                    /*	采用当前值替换规划序列第一行,初始化	*/
                    fJntP2PNodeDeg[0][k] = JntCurrentAngleDeg[k];
                }
            }
            else
            {
                /*	不能进行正常规划；期望关节赋值为当前关节角	*/
                PlanningSuccesFlg = 0;
                g_bFinishFlag = 1;
                for (i = 0; i < 7; i++)
                {
                    fDesiredJntAngleDeg[i] = JntCurrentAngleDeg[i];
                    fDesiredJntRateDeg[i] = 0.0f;
                    fDesiredJntAccDeg[i] = 0.0f;
                }
                return PlanningSuccesFlg;
            }
        }

        else
        {
            /*	当前位置与规划起始点不一致；期望关节赋值为当前关节角	*/
            g_bFinishFlag = 1;
            g_bCurrentIllEquInitErrorFlag = 1;
            PlanningSuccesFlg = 0;

            for (i = 0; i < 7; i++)
            {
                fDesiredJntAngleDeg[i] = JntCurrentAngleDeg[i];
                fDesiredJntRateDeg[i] = 0.0f;
                fDesiredJntAccDeg[i] = 0.0f;
            }

            return PlanningSuccesFlg;
        }

        g_iNodeNum_seq = -1;

    } /*	第一次规划处理结束	*/

    /*	运行时间	*/
    g_fPlanRunTime_OBC = cstarTime - g_fPlanStartTime_OBC; //=0

    /*	判断规划是否结束	*/
    if (g_iPlanFirstCircle > g_iPlanStepMax)
    {
        /*	规划结束	*/
        g_bFinishFlag = 1;
        PlanningSuccesFlg = 1;

        for (i = 0; i < 7; i++)
        {
            fDesiredJntAngleDeg[i] = fJntP2PNodeDeg[iNodeNum - 1][i];
            fDesiredJntRateDeg[i] = 0.0f;
            fDesiredJntAccDeg[i] = 0.0f;
        }
    }
    else
    {
        /*	规划未结束	*/
        if (g_bStartTimeIsZeros && g_bTimeSeqRight)
        {
            //*	五次多项式计算期望值	*/检查下ftime，目前靠规划算法内部定时，没用cstartime
            float fTime = 0;
            float qq_data[3];
            PlanningSuccesFlg = 1; /*	成功进行正常规划	*/

            /************************************************************************/
            /*						采用五次多项式进行规划                            */
            /************************************************************************/

            /*fTime为当前步所对应的时间	*/
            fTime = g_iPlanFirstCircle * g_fSampleTime_OBC; // 规划次数*采样时间

            if (fTime >= g_fTimeSeq[g_iNodeNum_seq + 1])
            {
                g_iNodeNum_seq++;
                for (k = 0; k < 7; k++)
                {
                    Interp5rdPoly_Param_P2P_Zrros_fcn(fJntP2PNodeDeg[g_iNodeNum_seq][k], fJntP2PNodeDeg[g_iNodeNum_seq + 1][k], g_fTimeSeq[g_iNodeNum_seq + 1] - g_fTimeSeq[g_iNodeNum_seq], g_a0345[k]);
                }
            }
            /*	判断规划的关节角度是否超限	*/
            for (k = 0; k < 7; k++)
            {
                Interp5rdPoly_Data_t_P2P_Zrros_fcn(fTime - g_fTimeSeq[g_iNodeNum_seq], g_a0345[k], qq_data);
                fTmpDesiredJntAngleDeg[k] = qq_data[0];
                fTmpDesiredJntRateDeg[k] = qq_data[1];
                fTmpDesiredJntAccDeg[k] = qq_data[2];
                if ((fTmpDesiredJntAngleDeg[k] < g_afJntAngleBoundaryDeg[0][k]) || (fTmpDesiredJntAngleDeg[k] > g_afJntAngleBoundaryDeg[1][k])) // Notice:关节速度超限保护与规划关节角和当前关节角的差值保护
                {
                    /*中间路径点有超出关节的运动范围，路径不可行标志，置1*/
                    g_bNoFeasiblePath = 1;
                    break;
                }
            }
            for (i = 0; i < 7; i++)
            {
                if (g_bNoFeasiblePath == 1) /*	超出关节范围，期望关节值赋值为当前角度	*/
                {
                    fDesiredJntAngleDeg[i] = JntCurrentAngleDeg[i];
                    fDesiredJntRateDeg[i] = 0.0f;
                    fDesiredJntAccDeg[i] = 0.0f;
                }
                else /*	未超出关节范围，期望关节值赋值为规划值	*/
                {
                    fDesiredJntAngleDeg[i] = fTmpDesiredJntAngleDeg[i];
                    fDesiredJntRateDeg[i] = fTmpDesiredJntRateDeg[i];
                    fDesiredJntAccDeg[i] = fTmpDesiredJntAccDeg[i];
                }
            }
        }
        /*	时间起始点或时间序列不正确，不能进行正常规划；返回当前关节角	*/
        else
        {
            PlanningSuccesFlg = 0;
            g_bFinishFlag = 1;
            for (i = 0; i < 7; i++)
            {
                fDesiredJntAngleDeg[i] = JntCurrentAngleDeg[i];
                fDesiredJntRateDeg[i] = 0.0f;
                fDesiredJntAccDeg[i] = 0.0f;
            }
        }
    }
    /*	规划次数 +1	*/
    g_iPlanFirstCircle++;
    return PlanningSuccesFlg;
}

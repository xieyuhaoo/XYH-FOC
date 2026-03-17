
#include "common.h"


_RAM_FUNC void DPWM_schedule_update(pmsm_t *pm)
{
    const int16_t arr      = (int16_t)PWM_ARR();
    const int16_t Tmin     = (int16_t)600;//设置最小采样窗口
    const int16_t Tmindiv2    = (int16_t)(0.5f * Tmin);
    const int32_t Tuplimit   = arr - 1;   /* CCR 上限 (对应TI Tuplimit) */
    const int32_t Tdownlimit = 1;         /* CCR 下限 (对应TI Tdownlimit) */
    const int16_t Tcontmini = 0;//0代表该相一直输出为高
    const int16_t Tcontmax = arr + 1;//代表该相一直输出为低

    int16_t Ton1U ; //上半周期A相占空比
    int16_t Ton2U ; //上半周期B相占空比
    int16_t Ton3U ; //上半周期C相占空比

    int16_t Ton1D ; //下半周期A相占空比
    int16_t Ton2D ; //下半周期B相占空比
    int16_t Ton3D ; //下半周期C相占空比

    uint16_t trig1;//第一次触发点
    uint16_t trig2;//第二次触发点

    pm->foc.shunt_case = 0;  /* 默认: 无移相 (Case A: T1和T2都足) */

    /* 归一化: 实际 αβ 电压 (V) × inv_vbus (√3/vbus) → SVM 归一化输入
     *   alpha, beta 归一化后使得 SVM 中的基本矢量运算能够正确匹配 Vbus
     *   因为 SVPWM 内切圆的最大有效相电压是 Vbus / √3, 
     *   如果输入相电压刚好也是 Vbus / √3，那么 (Vbus/√3) * (√3/Vbus) = 1.0
     *   正好对应占空比达到马鞍波的 1.0 上限 */
    float alpha = pm->foc.v_alph * pm->foc.inv_vbus;
    float beta  = pm->foc.v_beta * pm->foc.inv_vbus;

    int Sextant;
    float tmp1 = beta;
    float tmp2 = -0.5f * beta + SQRT3_BY_2 * alpha;
    float tmp3 = -tmp2 - tmp1;

    if (tmp3 > 0.0f)
    {
        if (tmp2 > 0.0f) Sextant = (tmp1 > 0.0f) ? 0 : 6;
        else             Sextant = (tmp1 > 0.0f) ? 5 : 4;
    }
    else
    {
        if (tmp2 > 0.0f) Sextant = (tmp1 > 0.0f) ? 3 : 2;
        else             Sextant = (tmp1 > 0.0f) ? 1 : 0;
    }
    int16_t X = (int16_t)(  beta * arr);
    int16_t Y = (int16_t)(( SQRT3_BY_2 * alpha + 0.5f * beta) * arr);
    int16_t Z = (int16_t)((0.5f * beta - SQRT3_BY_2 * alpha ) * arr);
    int16_t tA, tB, tC, t1, t2,Tdelta;
    switch (Sextant)
    {
    case 3:
    {
        t1 = (int16_t)( -Z );
        t2 = (int16_t)( X );
        if(t1>=t2)//DPWM1-0-30°: 窗口1(t2)不足,推B相下移
        {

            tA = Tcontmini;
            tB = tA + t1;
            tC = tB + t2;
            Ton1U= (int16_t)(tA);
            Ton2U= (int16_t)(tB);
            Ton3U= (int16_t)(tC);
            Ton1D= (int16_t)(tA);
            Ton2D= (int16_t)(tB);
            Ton3D= (int16_t)(tC);
            pm->foc.shunt_case = 1;
            if( ( t1 + t2 >= 2 * Tmin ) && ( t2  < Tmin ) )//有效矢量满足移相要求
             {
                Ton2U=Ton3U-Tmin;//调整第一阶段：仅调整B相，
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2U>Tuplimit)//B相向左移使右边沿到达最大值
                {
                    Ton2U=Tuplimit;
                    Ton2D=2*tB-Ton2U;
                    Ton3U=Ton2U+Tmin;//调整第三阶段：调整C相
                    Ton3D=2*tC-Ton3U;
                    pm->foc.shunt_case = 4;
                }
             }
             else if ( ( t1 + t2 < 2 * Tmin ) && ( (T1+T2)>= Tmini ))
            {
                Ton2U=Tmini;
                Ton2D=2*tB-Ton2U;//不会出现Ton2D < Tuplimit的情况
                Ton3U=Ton2U+Tmin;//调整第三阶段：调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 3;
            }
            else if ((T1+T2) < Tmini)
        {
                Ton2U=Tmini;
                Ton2D=2*tB-Ton2U;
                if(Ton2D<Tdownlimit)
                {
                    Ton2D=Tdownlimit;
                }
                Ton3U=Ton2U+Tmini;//调整第三阶段：调整C相
                if(Ton3D<Tdownlimit)
                {
                    Ton3D=Tdownlimit
                }
                pm->foc.shunt_case = 4;
        }
        trig1=(Ton2U)/2;
        trig2=(Ton3U+Ton2U)/2;
        }
        if(t1<t2)//DPWM1-30-60°: 窗口2(t1)不足,推A相下移
        {
            tC=Tcontmax;
            tB=tC-t2;
            tA=tB-t1;
            Ton1U= (int16_t)(tA);
            Ton2U= (int16_t)(tB);
            Ton3U= (int16_t)(tC);
            Ton1D= (int16_t)(tA);
            Ton2D= (int16_t)(tB);
            Ton3D= (int16_t)(tC);
            pm->foc.shunt_case = 1;
            if( ( t1 + t2 >= 2 * Tmin ) && ( t1  < Tmin ) )//有效矢量满足移相要求
            {
                Ton1U=Ton3U-Tmin;//调整第一阶段：仅调整A相，
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1U<Tdownlimit)//A相向右移使左边沿到达最小值
                {
                    Ton1U=Tdownlimit;
                    Ton1D=2*tA-Ton1U;
                    Ton2U=Ton1U+Tmin;//调整第二阶段：调整B相
                    Ton2D=2*tB-Ton2U;
                    pm->foc.shunt_case = 4;
                }
            }
            else if ( ( t1 + t2 < 2 * Tmin ) && ( (T1+T2)>= Tmini ))
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;//不会出现Ton1D < Tdownlimit的情况
                Ton2U=Ton1U+Tmin;//调整第二阶段：调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 3;
            }
            else if ((T1+T2) < Tmini)

        }

    } break;
    case 1:
    {
        t1 = (int16_t)( Z );
        t2 = (int16_t)( Y );
        tB = ( arr - t1 - t2) * 0.5f;
        tA = tB + t1;
        tC = tA + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
         if(t2 < Tmin && t1 > Tmin)//90-120度
         {
            Ton3U=tA+Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U>Tuplimit)//C相调整后仍不足，进入第二阶段：调整A相
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
                Ton1U=Tuplimit-Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D>Tuplimit)
                {
                    Ton1D=Tuplimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton2U=Ton2U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 > Tmin && t1 < Tmin)//60-90度
         {
            Ton2U=tA-Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
            pm->foc.shunt_case = 1;
            if(Ton2U<Tdownlimit)//B相调整后仍不足，进入第二阶段：调整C相
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;
                Ton1U=Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                    pm->foc.shunt_case = 2;
                if(Ton1D<Tdownlimit)//A相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton1D=Tdownlimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton2U=tA-Tmin;//调整第一阶段：调整B相，
            Ton2D=2*tB-Ton2U;
            Ton3U=tA+Tmin;//调整第三阶段：调整A相
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 4;
            if(Ton2U<Tdownlimit)
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;
            }
            if(Ton3U>Tuplimit)
            {
                Ton3U=Tuplimit;
                Ton3D=2*tC-Ton3U;
            }
         }
        trig1=(Ton2U+Ton1U)/2;
        trig2=(Ton3U+Ton1U)/2;
    } break;
    case 5:
    {
        t1 = (int16_t)(X);
        t2 = (int16_t)(-Y);
        tB = ( arr - t1 - t2) * 0.5f;
        tC = tB + t1;
        tA = tC + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 > Tmin)//120-150°: 窗口1(t2)不足,推A相上移
         {
            Ton1U=tC+Tmin;//Stage1: 仅调整A相上移
            Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tB错误 */
            pm->foc.shunt_case = 1;
            if(Ton1U>Tuplimit)//A相调整后仍不足,进入Stage2:调整C相
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tB错误 */
                Ton3U=Tuplimit-Tmin;//Stage2: 调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2;
                if(Ton3D>Tuplimit)
                {
                    Ton3D=Tuplimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton2U=Ton2U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 > Tmin && t1 < Tmin)//150-180°: 窗口2(t1)不足,推B相下移
         {
            Ton2U=tC-Tmin;//Stage1: 仅调整B相下移
            Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
            pm->foc.shunt_case = 1;
            if(Ton2U<Tdownlimit)//B相调整后仍不足,进入Stage2:调整C相
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
                Ton3U=Tmin;//Stage2: 调整C相为Tmin创建窗口
                Ton3D=2*tC-Ton3U;  /* C相中心=tC 正确 */
                pm->foc.shunt_case = 2;
                if(Ton3D<Tdownlimit)//C相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton3D=Tdownlimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制: 两窗口都不足,同时移相
         {
            Ton2U=tC-Tmin;//B相下移
            Ton2D=2*tB-Ton2U;  /* FIX: B相中心=tB,原2*tC错误 */
            Ton1U=tC+Tmin;//A相上移
            Ton1D=2*tA-Ton1U;  /* FIX: A相中心=tA,原2*tC错误 */
            pm->foc.shunt_case = 4;
            if(Ton2U<Tdownlimit)
            {
                Ton2U=Tdownlimit;
                Ton2D=2*tB-Ton2U;  /* FIX */
            }
            if(Ton1U>Tuplimit)
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;  /* FIX */
            }
         }
        trig1=(Ton2U+Ton3U)/2;
        trig2=(Ton3U+Ton1U)/2;
    } break;
    case 4:
    {
        t1 = (int16_t)(-X);
        t2 = (int16_t)(Z);
        tC = ( arr - t1 - t2) * 0.5f;
        tB = tC + t1;
        tA = tB + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 > Tmin)//210-240度
         {
            Ton1U=tB+Tmin;//调整第一阶段：仅调整A相，
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 1;
            if(Ton1U>Tuplimit)//A相调整后仍不足，进入第二阶段：调整B相
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;
                Ton2U=Tuplimit-Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D>Tuplimit)//B相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tuplimit;
                    Ton2U=2*tB-Ton2D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    Ton2U=Ton2U-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }

            }
         }
         else if (t2 > Tmin && t1 < Tmin)//180-210度
         {
            Ton3U=tB-Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U<Tdownlimit)//C相调整后仍不足，进入第二阶段：调整B相
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                Ton2U=Tmin;//调整B相
                Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 2;
                if(Ton2D<Tdownlimit)//B相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton2D=Tdownlimit;
                    Ton2U=2*tB-Ton2D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
            
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton3U=tB-Tmin;//调整第一阶段：调整C相，
            Ton3D=2*tC-Ton3U;
            Ton1U=tB+Tmin;//调整第三阶段：调整A相
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 4;
            if(Ton3U<Tdownlimit)
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
            }
            if(Ton1U>Tuplimit)
            {
                Ton1U=Tuplimit;
                Ton1D=2*tA-Ton1U;
            }
         }
        trig1=(Ton2U+Ton3U)/2;
        trig2=(Ton2U+Ton1U)/2;
    } break;
    case 6:
    {
        t1 = (int16_t)(-Y);
        t2 = (int16_t)(-Z);
        tC = ( arr - t1 - t2) * 0.5f;
        tA = tC + t1;
        tB = tA + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
         if(t2 < Tmin && t1 > Tmin)//240-270度
         {
            Ton2U=tA+Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 1;
            if(Ton2U>Tuplimit)//B相调整后仍不足，进入第二阶段：调整C相
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                Ton1U=Tuplimit-Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D>Tuplimit)
                {
                    Ton1D=Tuplimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton3U=Ton3U-0.5*Tdelta;
                    Ton3D=Ton3D-0.5*Tdelta;
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }

         }
         else if (t2 > Tmin && t1 < Tmin)//270-300度
         {
            Ton3U=tA-Tmin;//调整第一阶段：仅调整C相，
            Ton3D=2*tC-Ton3U;
            pm->foc.shunt_case = 1;
            if(Ton3U<Tdownlimit)//C相调整后仍不足，进入第二阶段：调整B相
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
                Ton1U=Tmin;//调整A相
                Ton1D=2*tA-Ton1U;
                pm->foc.shunt_case = 2;
                if(Ton1D<Tdownlimit)//A相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton1D=Tdownlimit;
                    Ton1U=2*tA-Ton1D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    Ton1U=Ton1U+0.5*Tdelta;
                    Ton3D=Ton3D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton3U=tA-Tmin;//调整第一阶段：调整C相，
            Ton3D=2*tC-Ton3U;
            Ton2U=tA+Tmin;//调整第三阶段：调整B相
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 4;
            if(Ton3U<Tdownlimit)
            {
                Ton3U=Tdownlimit;
                Ton3D=2*tC-Ton3U;
            }
            if(Ton2U>Tuplimit)
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
            }
            
         }
        trig1=(Ton3U+Ton1U)/2;
        trig2=(Ton1U+Ton2U)/2;
    } break;
    case 2:
    default:
    {
        int16_t t1 = (int16_t)(Y);
        int16_t t2 = (int16_t)(-X);
        tA = (arr - t1 - t2) * 0.5f;
        tC = tA + t1;
        tB = tC + t2;
        Ton1U= (int16_t)(tA);
        Ton2U= (int16_t)(tB);   
        Ton3U= (int16_t)(tC);
        Ton1D= (int16_t)(tA);
        Ton2D= (int16_t)(tB); 
        Ton3D= (int16_t)(tC);
        if(t2 < Tmin && t1 > Tmin)//330-360度
         {
            Ton2U=tC+Tmin;//调整第一阶段：仅调整B相，
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 1;
            if(Ton2U>Tuplimit)//B相调整后仍不足，进入第二阶段：调整A相
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
                Ton3U=Tuplimit-Tmin;//调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2;
                if(Ton3D>Tuplimit)
                {
                    Ton3D=Tuplimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t1);//计算不足量
                    Ton1U=Ton1U-0.5*Tdelta;
                    Ton1D=Ton1D-0.5*Tdelta;
                    Ton2D=Ton2D-0.5*Tdelta;
                    Ton3U=Ton3U-0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t2 > Tmin && t1 < Tmin)//300-330度
         {
            Ton1U=tC-Tmin;//调整第一阶段：仅调整A相，
            Ton1D=2*tA-Ton1U;
            pm->foc.shunt_case = 1;
            if(Ton1U<Tdownlimit)//A相调整后仍不足，进入第二阶段：调整A相
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
                Ton3U=Tmin;//调整C相
                Ton3D=2*tC-Ton3U;
                pm->foc.shunt_case = 2; 
                if(Ton3D<Tdownlimit)//C相调整后仍不足，进入第三阶段：调整A,B,C三相
                {
                    Ton3D=Tdownlimit;
                    Ton3U=2*tC-Ton3D;
                    Tdelta=Tmin-(arr-t2);//计算剩余不足量
                    Ton2U=Ton2U+0.5*Tdelta;
                    Ton2D=Ton2D+0.5*Tdelta;
                    Ton3U=Ton3U+0.5*Tdelta;
                    Ton1D=Ton1D+0.5*Tdelta;
                    pm->foc.shunt_case = 3;
                }
            }
         }
         else if (t1 < Tmin && t2 < Tmin)//低调制
         {
            Ton1U=tC-Tmin;//调整第一阶段：调整A相，
            Ton1D=2*tA-Ton1U;
            Ton2U=tC+Tmin;//调整第三阶段：调整B相
            Ton2D=2*tB-Ton2U;
                pm->foc.shunt_case = 4;
            if(Ton1U<Tdownlimit)
            {
                Ton1U=Tdownlimit;
                Ton1D=2*tA-Ton1U;
            }
            if(Ton2U>Tuplimit)
            {
                Ton2U=Tuplimit;
                Ton2D=2*tB-Ton2U;
            }
         }
        trig1=(Ton1U+Ton3U)/2;
        trig2=(Ton3U+Ton2U)/2;
    } break;
    }


    pm->foc.svm_sector = (uint8_t)Sextant;

    // float t1_norm = (Sextant & 1) ? tv2 : tv1;
    // float t2_norm = (Sextant & 1) ? tv1 : tv2;
    // t1_norm = foc_clamp_f32(t1_norm, 0.0f, 1.0f);
    // t2_norm = foc_clamp_f32(t2_norm, 0.0f, 1.0f);
    // pm->foc.t1_eff = t1_norm;
    // pm->foc.t2_eff = t2_norm;


    pm->foc.arr1_up = (uint16_t)Ton1U;
    pm->foc.arr2_up = (uint16_t)Ton2U;
    pm->foc.arr3_up = (uint16_t)Ton3U;
    pm->foc.arr1_down = (uint16_t)Ton1D;
    pm->foc.arr2_down = (uint16_t)Ton2D;
    pm->foc.arr3_down = (uint16_t)Ton3D;





    pm->foc.sample_trig1 = (uint16_t)trig1;
    pm->foc.sample_trig2 = (uint16_t)trig2;

    /* TODO: 这里预留给你写移相(SSPS)逻辑，基于 t1_eff/t2_eff + sector 直接修改 arr*_up/down 和 sample_trig* */
}
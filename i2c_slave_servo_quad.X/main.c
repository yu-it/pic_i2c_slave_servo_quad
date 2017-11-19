/* 
 * File:   main.c
 * Author: yuusuke.ito
 *
 * 
 * 
 * 
 * 1  --VDD         |  20 VSS
 * 2  --RA5         |  19 RA0
 * 3  --RA4         |  18 RA1
 * 4  --RA3         |  17 RA2 pwm3  X   
 * 5  --RC5 pwm1    |  16 RC0 
 * 6  --RC4         |  15 RC1 pwm4  
 * 7  --RC3 pwm2    |  14 RC2 
 * 8  --RC6         |  13 RB4 SDA
 * 9  --RC7         |  12 RB5 
 * 10 --RB7 PWM Con |  11 RB6 SCL
 * 
 * 17はモータのパワーコントローラ * 
 * i2cset -y 1 [ADD] [mem_offset] [command] [value] [ex-value]
 * i2cset -y 1 0x12 0x00 0x0a 0x11
 * Created on 2017/04/01, 11:25
 */
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <pic16F1508.h>
#include "main.h"
/***** コンフィギュレーションの設定 *********/
/*
__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_ON & CP_OFF
	& BOREN_ON & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_OFF & STVREN_OFF & LVP_OFF);
*/
// コンフィギュレーション１の設定
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset) aa
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)aaa
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled) zzz

/** グローバル定数変数定義 **/
unsigned int HWidth, VWidth, DFlag, Mode, i;
unsigned int Interval, AutoFlag, AutoInt, Level, NewLvl;
const unsigned int MAX = 1000;
const unsigned int MIN = 250;
//#define  _XTAL_FREQ     8000000      // クロック周波数500kHz
unsigned int current_1, current_4;


/*volatile*/ int arm_mode = 0;    
typedef struct {
    servo_descriptor* servo1;
    servo_descriptor* servo2;
    servo_descriptor* servo3;
    servo_descriptor* servo4;
    
} mech_descriptor;

servo_descriptor servo1_t;
servo_descriptor servo2_t;
servo_descriptor servo3_t;
servo_descriptor servo4_t;
mech_descriptor mech_desc_t;
mech_descriptor* mech_desc = &mech_desc_t;

/*
 * set_register_flg2
 * レジスタのフラグを設定する
 * 第一引数にレジスタアドレス、第二引数に何番目のフラグに立てるか、第三引数に値をセット
 * もともと1が立っているところにまた1をセットしたりするとどういうわけか出力が一瞬LOWになってしまう。値が異なるときのみフラグをセットするようにしている。
 */
void set_register_flg(volatile unsigned char * regaddr, int regoffset, unsigned int val) {
    
    BitIndicator* tar =  (BitIndicator * ) (regaddr);
    if (0 == regoffset) {
        if (tar->B0 != val)tar->B0 = val;   
    } else if (1 == regoffset) {
        if (tar->B1 != val)tar->B1 = val;
    } else if (2 == regoffset) {
        if (tar->B2 != val)tar->B2 = val;
    }else if (3 == regoffset) {
        if (tar->B3 != val)tar->B3 = val;
    }else if (4 == regoffset) {
        if (tar->B4 != val)tar->B4 = val;
    }else if (5 == regoffset) {
        if (tar->B5 != val)tar->B5 = val;
    }else if (6 == regoffset) {
        if (tar->B6 != val)tar->B6 = val;
    }else if (7 == regoffset) {
        if (tar->B7 != val)tar->B7 = val;
    }
}   

void init_servo_struct(servo_descriptor* servo) {
    *(servo->PWMH_register) = 0;
    *(servo->PWML_register) = 0;
    servo->current_freq = 0;
    servo->actual_outputting_freq = 0;
    servo->dir = 2;
    servo->max_frequency = 0;
    servo->min_frequency = 0;
    servo->power = 0;
    servo->power_control_work = 0;

}
void init_mortor_struct(mortor_descriptor* mortor) {
    *(mortor->LAT_register_for_dir_control1) = 0;
    mortor->LAT_register_for_dir_control1_offset = 0;
    *(mortor->LAT_register_for_dir_control2) = 0;
    mortor->LAT_register_for_dir_control2_offset = 0;
    *(mortor->PWMH_register_for_power_control) = 0;
    *(mortor->PWML_register_for_power_control) = 0;
    mortor->dir = 2;
    mortor->power = 0xff90;

}
void aply_servo_freq(servo_descriptor* servo) {
    *(servo->PWMH_register) = (servo->actual_outputting_freq) >> 2;
    *(servo->PWML_register) = (servo->actual_outputting_freq) << 6;
}
void aply_mor_power(mortor_descriptor* mor) {
    *(mor->PWMH_register_for_power_control) = mor->power >> 2;
    *(mor->PWML_register_for_power_control) = mor->power << 6;
}
void aply_mor_dir(mortor_descriptor* mor) {
    if (3 == mor->dir) {
        set_register_flg(mor->LAT_register_for_dir_control1, mor->LAT_register_for_dir_control1_offset, 1);
        set_register_flg(mor->LAT_register_for_dir_control2, mor->LAT_register_for_dir_control2_offset, 0);
    } else if (1 == mor->dir) {
        set_register_flg(mor->LAT_register_for_dir_control1, mor->LAT_register_for_dir_control1_offset, 0);
        set_register_flg(mor->LAT_register_for_dir_control2, mor->LAT_register_for_dir_control2_offset, 1);
    } else if (2 == mor->dir) {
        set_register_flg(mor->LAT_register_for_dir_control1, mor->LAT_register_for_dir_control1_offset, 1);
        set_register_flg(mor->LAT_register_for_dir_control2, mor->LAT_register_for_dir_control2_offset, 1);
    }
    
}
void set_servo_min(servo_descriptor* servo, int min) {
    if (servo->min_frequency != min)servo->min_frequency = min;
    if(servo->current_freq < min) {
        servo->current_freq = min;
    }
}
void set_servo_max(servo_descriptor* servo, int max) {
    if (servo->max_frequency != max)servo->max_frequency = max;
    if(servo->current_freq > max) {
        servo->current_freq = max;
    }
}
void set_servo_dir(servo_descriptor* servo, int dir) {
    if (servo->dir != dir)servo->dir = dir;
}
void set_servo_pow(servo_descriptor* servo, int pow) {
    if (servo->power != pow)servo->power = pow;
}
void calc_desired_servo_status(servo_descriptor* servo) {
    
    //power変数を用いて周波数を変えるスピードをコントロールする。
    servo->power_control_work ++ ;
    if (servo->power_control_work < servo->power) {
        return;
    }
    servo->power_control_work = 0;

    //servo更新
    if ((servo->dir - 2) == 0) {
         //現在の出力周波数は保持しつつ、サーボを止めるために実際に出力する周波数が格納されているactual～のほうを0にする。
        servo->actual_outputting_freq = 0; 
    } else {
        servo->current_freq += servo->dir - 2;

        //行き過ぎている場合は補正する
        if (servo->current_freq < servo->min_frequency) {
            servo->current_freq = servo->min_frequency;
        } else if (servo->current_freq > servo->max_frequency) {
            servo->current_freq = servo->max_frequency;
        }            

        servo->actual_outputting_freq = servo->current_freq;
    }
        
}

void set_mor_dir(mortor_descriptor* mor, int dir) {
    if (mor->dir != dir)mor->dir = dir;
}
void set_mor_power(mortor_descriptor* mor, int pow) {
    if (mor->power != pow)mor->power = pow;
}


/*
 * 
 */

void Init() {
    OSCCON = 0x73;                      // 内蔵8MHz
    /** 入出力ポートの設定 ***/
    ANSELA = 0x00;
    ANSELB = 0x00;                      
    ANSELC = 0x00;                      
    TRISA = 0;                       // すべて出力
    TRISB = 0;                       // すべて一旦出力
    TRISC = 0;                       // すべて出力
    
    //TRISCbits.TRISC2 = 1;            //センサー1
    //TRISBbits.TRISB5 = 1;            //センサー2

    /** プルアップイネーブル **/
    WPUA = 0b0;
    /* タイマ0の設定　20.04msec周期 */
    OPTION_REG = 0x07;                  // Int. 1/256, プルアップ有効化
    TMR0 = 99;                          // 20msec         
    /* PWM1,4の初期設定 2.048msec周期*/
    /* Duty値設定範囲  450 to 1023 (0.9ms to 2.05ms width) */
    
    PWM1CON = 0xC0;                     // PWM1オン OutputOn
    PWM4CON = 0xC0;                     // PWM4オン OutputOn
    PWM2CON = 0xC0;                     // PWM3オン OutputOn
    PWM3CON = 0xC0;                     // PWM1オン OutputOn

    /* タイマ２の設定 */
    T2CON = 0x06;                       // 1/16  2MHz/16/256 = 2.048msec周期
    PR2 = 0xFF;                         // 10bit分解能
    /** CLCの初期設定  **/
    CLCInit();
     /* 変数リセット */
    /* 割り込み許可 */
    TMR0IE = 1;                         // タイマ0割り込み許可
    //TMR2IE = 1;                         // タイマ0割り込み許可
    PEIE = 1;                           // 周辺許可
    GIE = 1;                            // グローバル許可
    setUpI2CSlave();
    PWM1DCH = 0xffff >> 2;
    PWM1DCL = 0xffff << 6;
    PWM2DCH = 0xffff >> 2;
    PWM2DCL = 0xffff << 6;
    PWM3DCH = 0xffff >> 2;
    PWM3DCL = 0xffff << 6;
    PWM4DCH = 0xffff >> 2;
    PWM4DCL = 0xffff << 6;
}
void init_struct() {
    //構造体の初期化
    mech_desc->servo1 = &servo1_t;
    mech_desc->servo2 = &servo2_t;
    mech_desc->servo3 = &servo3_t;
    mech_desc->servo4 = &servo4_t;
    
    init_servo_struct(mech_desc->servo1);
    init_servo_struct(mech_desc->servo2);
    init_servo_struct(mech_desc->servo3);
    init_servo_struct(mech_desc->servo4);
   
    //レジスタとの関連付け
    mech_desc->servo1->PWML_register = &PWM1DCL;
    mech_desc->servo1->PWMH_register = &PWM1DCH;
    mech_desc->servo2->PWML_register = &PWM4DCL;
    mech_desc->servo2->PWMH_register = &PWM4DCH;
    mech_desc->servo3->PWML_register = &PWM2DCL;
    mech_desc->servo3->PWMH_register = &PWM2DCH;
    mech_desc->servo4->PWML_register = &PWM3DCL;
    mech_desc->servo4->PWMH_register = &PWM3DCH;

    
}
int main(int argc, char** argv) {

    Init();
    init_struct();
    while(1) {
        
        if (arm_mode) {
            optimize_arm_angle();
        }
        calc_desired_status();
        apply_desired_status();
        
    }
}

/*************************************
* CLC初期設定関数
*************************************/
void CLCInit(void){
   /* Timer0の20msec周期でPWM3とPWM4の出力を1回のみ有効化 */
    /* CLC1 初期設定 */
    CLC1GLS0 = 0x08;
    CLC1GLS1 = 0x00;                    // RSフリップフロップ
    CLC1GLS2 = 0x02;                    // Timer0でセット
    CLC1GLS3 = 0x00;                    // Timer2でリセット
    CLC1SEL0 = 0x17;
    CLC1SEL1 = 0x60;
    CLC1POL  = 0x00;
    CLC1CON  = 0xC3;
    /* CLC2 初期設定 */
    CLC2GLS0 = 0x08;
    CLC2GLS1 = 0x20;                    // AND回路
    CLC2GLS2 = 0x00;                    // CLC3出力とPWM1出力のAND
    CLC2GLS3 = 0x00;
    CLC2SEL0 = 0x62;
    CLC2SEL1 = 0x66;
    CLC2POL  = 0x0C;
    CLC2CON  = 0xC2;
    /* CLC3 初期設定 */
    CLC3GLS0 = 0x02;
    CLC3GLS1 = 0x08;                    // Dタイプフリップフロップ
    CLC3GLS2 = 0x00;                    // CLC1の出力をD入力
    CLC3GLS3 = 0x00;                    // Timer2がクロック
    CLC3SEL0 = 0x47;
    CLC3SEL1 = 0x50;
    CLC3POL  = 0x00;
    //CLC3CON  = 0x84;
    CLC3CON  = 0xC4;
    
    /* CLC4 初期設定 */
    CLC4GLS0 = 0x08;
    CLC4GLS1 = 0x80;                    // AND回路
    CLC4GLS2 = 0x00;                    // CLC3出力とPWM4出力のAND
    CLC4GLS3 = 0x00;
    CLC4SEL0 = 0x62;
    CLC4SEL1 = 0x37;
    CLC4POL  = 0x0C;
    CLC4CON  = 0xC2;
}

void interrupt isr(void){
    if(TMR0IF){                             // タイマ0割り込みか？
        TMR0IF = 0;                         // フラグクリア
        TMR0 = 96;                          // 時間再設定
    }
    I2Cinterrupt();
    //PORTC6
    //PORTC7
    
}

void i2c_handler_impl(unsigned int com, unsigned int data) {

         if (com == set_servo1min_com) {
             set_servo_min(mech_desc->servo1, data);
         } else if (com == set_servo1max_com) {
             set_servo_max(mech_desc->servo1, data);
         } else if (com == set_servo2min_com) {
             set_servo_min(mech_desc->servo2, data);
         } else if (com == set_servo2max_com) {
             set_servo_max(mech_desc->servo2, data);
         } else if (com == set_servo3min_com) {
             set_servo_min(mech_desc->servo3, data);
         } else if (com == set_servo3max_com) {
             set_servo_max(mech_desc->servo3, data);
         } else if (com == set_servo4min_com) {
             set_servo_min(mech_desc->servo4, data);
         } else if (com == set_servo4max_com) {
             set_servo_max(mech_desc->servo4, data);
         }else if (com == set_servo1dir_com) {
             set_servo_dir(mech_desc->servo1, data);
         } else if (com == set_servo2dir_com) {
             set_servo_dir(mech_desc->servo2, data);
         }else if (com == set_servo3dir_com) {
             set_servo_dir(mech_desc->servo3, data);
         } else if (com == set_servo4dir_com) {
             set_servo_dir(mech_desc->servo4, data);
         } else if (com == set_servo1pow_com) {
             set_servo_pow(mech_desc->servo1, data);
         } else if (com == set_servo2pow_com) {
             set_servo_pow(mech_desc->servo2, data);
         } else if (com == set_servo3pow_com) {
             set_servo_pow(mech_desc->servo3, data);
         } else if (com == set_servo4pow_com) {
             set_servo_pow(mech_desc->servo4, data);
         }
 }
void optimize_arm_angle(){};
void calc_desired_status(){
    calc_desired_servo_status(mech_desc->servo1);
    calc_desired_servo_status(mech_desc->servo2);
    calc_desired_servo_status(mech_desc->servo3);
    calc_desired_servo_status(mech_desc->servo4);
    
};

void apply_desired_status(){
    aply_servo_freq(mech_desc->servo1);
    aply_servo_freq(mech_desc->servo2);
    aply_servo_freq(mech_desc->servo3);
    aply_servo_freq(mech_desc->servo4);
 
};




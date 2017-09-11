#include <stdlib.h>
#include <string.h>
#include <stm32f10x_conf.h>
#include "systickTimer.h"
#include "serialUart.h"
#include "i2c.h"
#include "timerPwm.h"
#include "vcom.h"

// A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x, y, z

red_vcomPort_t* redVcom;

void assert_failed(uint8_t* file, uint32_t line)
{
    if (redVcom != NULL)
        redVcom->printf(redVcom, "Assert fail at File %s Line %d\r\n", file,
                (uint16_t) line);
    while (1)
        ;
}

#define SOURCE_RAMDOM ((uint32_t)2061428883)

// seconds
#define READY_TIME 30
#define SHOW_PASSWORD_TIME 30
#define WAIT_TIME 172800 // (1209600/7) // 2weeks

//#define READY_TIME 5
//#define SHOW_PASSWORD_TIME 5
//#define WAIT_TIME 15

#define PASSWORD_DIGIT 8

#define CONFIG_SIZE 3
typedef union
{
    struct
    {
        char passwd[PASSWORD_DIGIT];
        uint8_t state;
        uint8_t checkSum;
    } s;
    uint32_t data[CONFIG_SIZE];
} config_t;

enum STATE
{
    READY = 0, WAIT, DONE, FORMAt
};

typedef struct
{
    uint8_t numOfNumber;
    uint8_t numOfCapital;
} passwordInfo_t;

const char pwTable[] =
{ '1', '2', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
        'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '0', 'A', 'B',
        '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', 'g',
        'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u',
        'v', 'w', 'x', 'y', 'z' };

const char *stateString[] =
{
       "READY", "WAIT", "DONE", "FORMAt"
};

#define FLASH_INDEX					((uint8_t)(SOURCE_RAMDOM % 63)+1) // switched by 1

#define FLASH_PAGE_COUNT                ((uint8_t)64)
#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - FLASH_INDEX))       // use the last 1 KB for storage

uint8_t write_flash(uint32_t *dataFrom, uint8_t numOfData)
{
    uint8_t err = 0, i;
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_WRITE_ADDR);
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    for (i = 0; i < numOfData; i++)
        err |= FLASH_ProgramWord((FLASH_WRITE_ADDR + i*4), dataFrom[i]);
    FLASH_Lock();
    return err;
}

uint8_t read_flash(uint32_t *dataTo, uint8_t numOfData)
{
    uint8_t i;
    for(i = 0; i < numOfData; i++)
        dataTo[i] = *(__IO uint32_t *)(FLASH_WRITE_ADDR + i*4);
    return 0;
}

uint8_t reset_flash(config_t *conf)
{
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_WRITE_ADDR);
//    FLASH_EraseAllPages();
    FLASH_Lock();
    conf->s.state = READY;
    return write_flash(conf->data, CONFIG_SIZE);
}

uint8_t save_passwd(config_t *conf, char *pw)
{
    memcpy(conf->s.passwd, pw, sizeof(conf->s.passwd));
    return write_flash(conf->data, CONFIG_SIZE);
}

uint8_t save_state(config_t *conf, uint8_t newState)
{
    conf->s.state = newState;
    return write_flash(conf->data, CONFIG_SIZE);
}

uint8_t load_config(config_t *conf)
{
    return read_flash(conf->data, CONFIG_SIZE);
}

passwordInfo_t generate_password(char *dataTo)
{
    uint8_t nandNum, i, j, unique;
    char nandChar = '\0';
    passwordInfo_t info;
    info.numOfCapital = 0;
    info.numOfNumber = 0;

    for(i = 0; i < PASSWORD_DIGIT; i++)
    {
        nandNum = (SysTick->VAL & 0x0000003F)%62;
        nandChar = pwTable[nandNum];

        unique = 0;
        for(j = 0; j < i; j++)
        {
            unique++;
            if(dataTo[i] == nandChar)
                break;
        }

        if(unique != i)
        {
            i--;
            continue;
        }

        dataTo[i] = nandChar;

        if(nandChar >= 'A' && nandChar <= 'Z')
            info.numOfCapital++;
        else if(nandChar >= '0' && nandChar <= '9')
            info.numOfNumber++;

        delay(100);
    }
    return info;
}

int main(void)
{
    systickTimerConfig();

    // Init vcomPort
    red_vcom_userSetting_t redVcomUserSetting = {
        .vcomMode = RED_VCOM_INTERRUPT_MODE,
    };
    redVcom = redVcomInit(RED_VCOM_PORT, &redVcomUserSetting);

    // LedBlink
    // Config LED Pin(PC13)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);

    GPIO_WriteBit(GPIOC, GPIO_Pin_13, SET);

    delay(500);

    //

    config_t config;
    load_config(&config);

    uint32_t t, tStart = 0;
    char pw[PASSWORD_DIGIT];
    passwordInfo_t pwInfo;
    uint8_t i;

    t = millis();
    tStart = t;

    redVcom->printf(redVcom, "\r\n\r\nProtector MCCXX %s\r\n\r\n", stateString[config.s.state]);

    char inChar[8] = { 0, };

    while (1)
    {
        while(redVcom->available(redVcom))
        {
            inChar[0] = redVcom->getChar(redVcom);
            if((inChar[0] ==  'F') && (config.s.state == DONE))
            {
                for(i = 1; i < 6; i++)
                {
                    if(redVcom->available(redVcom))
                        inChar[i] = redVcom->getChar(redVcom);
                    else
                        break;

                    if(i == 5)
                    {
                        inChar[6] = '\0';
                        if(strcmp(inChar,"FORMAT") == 0)
                            save_state(&config, FORMAt);
                    }
                }
            }
            else if((inChar[0] == 'L') && (config.s.state == WAIT))
            {
                for(i = 1; i < 7; i++)
                {
                    if(redVcom->available(redVcom))
                        inChar[i] = redVcom->getChar(redVcom);
                    else
                        break;

                    if(i == 6)
                    {
                        inChar[7] = '\0';
                        if(strcmp(inChar,"Liberte") == 0)
                            save_state(&config, FORMAt);
                    }
                }
            }
        }
        switch(config.s.state)
        {
        case READY:
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, RESET);
            if((millis()-t) < 1000 * READY_TIME)
            {
                redVcom->printf(redVcom, "Ready to Generate Password...%d\r\n", ((millis()-tStart)/1000));
                delay(1000);
            }
            else
            {
                uint8_t j = 0;
                while(true) // FIXME: avoid infinite loop
                {
                    pwInfo = generate_password(pw);
                    redVcom->printf(redVcom, "Generate Password...%d : ", ++j);
                    for(i = 0; i < PASSWORD_DIGIT; i++)
                        redVcom->printf(redVcom, "%c", pw[i]);
                    redVcom->printf(redVcom, " \r\n");
                    if(pwInfo.numOfNumber > 2 && pwInfo.numOfCapital > 1) // at least 2 number and 1 Capital charicter is needed
                        break;
                    delay(333);
                }

                save_passwd(&config, pw);
                redVcom->printf(redVcom, "\r\nAfter using below password, must be RESET device to proceed next step\r\n\r\n");
                t = millis();
                save_state(&config, WAIT); // #REPOWER
                while(true) // #REPOWER true -> (millis()-t) < 1000 * SHOW_PASSWORD_TIME
                {
                    redVcom->printf(redVcom, "# ");
                    delay(250);
                    for(i = 0; i < PASSWORD_DIGIT; i++)
                    {
                        redVcom->printf(redVcom, "%c", config.s.passwd[i]);
                        delay(250);
                    }
                    redVcom->printf(redVcom, " #\r\n");
                    delay(5000);
                }
                // To show password after time, you have to repower your mcu
//                t = millis();
//                tStart = t;
//                save_state(&config, WAIT);
            }
            break;

        case WAIT:
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, SET);
            if((millis()-t) < 1000 * WAIT_TIME)
            {
                redVcom->printf(redVcom, "%d\r\n", ((millis()-tStart)/1000));
                delay(5000);
            }
            else
            {
                redVcom->printf(redVcom, "\r\nDone\r\n\r\n");
                redVcom->printf(redVcom, "Use this password for Liberté\r\n\r\n");
                save_state(&config, DONE);
            }
            break;

        case DONE:
            for(i = 0; i < PASSWORD_DIGIT; i++)
            {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, i%2);
                redVcom->printf(redVcom, "%c", config.s.passwd[i]);
                delay(333);
            }
            redVcom->printf(redVcom, "\r\n");
            delay(1000);
            break;

        default:
        case FORMAt:
            redVcom->printf(redVcom, "\r\n");
            redVcom->printf(redVcom, "Format...\r\n");
            reset_flash(&config);
            delay(1000);
            redVcom->printf(redVcom, "Done\r\n\r\n");
            t = millis();
            tStart = t;
            break;
        }
    }
}

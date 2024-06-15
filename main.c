#include "msp.h"
#include ".\Clock.h"
#include "stdio.h"
#include "stdlib.h"

#define TURN_TIME_MS 500 //도는 시간
#define DOT_DETECT_DELAY 60000

#define nstartline 0
#define ndot 1
#define nline 2
#define nfinishline 3

int count = -1; //현재 몇번 점에 있는지 알려줄 카운트
int status = -1; //현재 상태를 알려줄 변수
int prev_status = -1; //직전  status를 알려줄 변수

int dot_detect_counter = 0;

int graph[8][8]; //그래프를 저장하는 2차원 배열. 값이 -1이면 연결되지 않은 것이고, 0~3의 숫자가 쓰여있다면 간선이 있는 것이다.
int Euler_path[16][2]; //한붓그리기 경로를 저장하는 배열. 첫칸에는 현재 점이,둘째칸에는 간선의 숫자가 들어간다.
int path_num =0;

void (*TimerA2Task) (void);
void TimerA2_Init (void(*task) (void), uint16_t period);
void task(void);
void TA2_0_IRQHandler(void);
void PWM_Init34(uint16_t period, uint16_t duty3,uint16_t duty4);
void LED_Init(void);
void IR_Init(void);
void Motor_Init(void);
void Turn_on_IR();
void PWM_Duty3(uint16_t duty3);
void PWM_Duty4(uint16_t duty4);

void Move(uint16_t leftDuty, uint16_t rightDuty);
void Left_Forward();
void Left_Backward();
void Right_Forward();
void Right_Backward();

void Left_rotate_45_degrees();
void Right_rotate_45_degrees();
void Left_rotate_90_degrees();
void Right_rotate_90_degrees();
void Left_rotate_135_degrees();
void Right_rotate_135_degrees();
void Right_rotate_135_degrees_p2();

void Rotate_Clock(int targetLine);
void Rotate_CounterClock(int targetLine);
void Rotate_CounterClock_p1(int targetLine);

int Count_edge(int index);

void find();
void startline();
void dot();
void line();

void reverse_find();
void Back_line();

int FindEuler(int vertex);

int main(void) {
    Clock_Init48MHz();
    LED_Init();
    IR_Init();
    Motor_Init();

    if (count==-1){
        //첫 점을 찾을 때까지 find를 돌려 이동하게 한다.
        while(status!=ndot)
        {
            find();
        }

        Left_Forward();
        Right_Forward();
        Move(3000, 3000);
        Clock_Delay1ms(160);
        Move(0, 0);

        //135도 회전한다.
        Right_rotate_135_degrees();
        count  = 0;
    }

    /* ========== Phase 1 - 트랙 탐색 ========== */

    int vertex_list[8][4]; // 8개의 점에 대한 정보를 저장하는 이차원 배열
    int vertex_length[8]; // 각각의 점에 몇 개의 선이 있는지

    int i;
    for(i=0; i<8; i++) { // 각 점에 연결 된 선 개수 확인
        if(i != 0) {
            // 회전하며 선 개수를 확인하기 위해 위치 세팅
            // 하드웨어 오류를 최소화하기 위한 위치 보정
            Left_Forward();
            Right_Forward();
            Move(3000, 3000);

            if(i < 3) {
                Clock_Delay1ms(190);
            }
            else {
                Clock_Delay1ms(210);
            }

            Right_rotate_90_degrees();
            Move(0,0);
            Clock_Delay1ms(150);

            Left_Backward();
            Right_Backward();
            Move(3000, 3000);
            Clock_Delay1ms(65);

            Move(0,0);
        }

        int edgeCnt = Count_edge(i);
        vertex_length[i] = edgeCnt;

        status = -1;
        Move(0,0);
        Clock_Delay1ms(300);

        if (edgeCnt != 4) {
            P2->OUT &= ~0x07;
            P2->OUT |= 0x03;
            Move(0,0);

            return 0;
        }

        int vertex[edgeCnt]; // 연결된 점을 저장하기 위한 배열

        //연결된 점들에게 인덱스 부여
        int j;
        int w = 1; // 가중치
        for(j=0; j<edgeCnt; j++) {
            if(j==edgeCnt-1) {
                vertex[j] = i-1;
            }
            vertex[j] = i + w; //i+1 | i+3 | i+5
            w += 2;

            if(vertex[j] == -1) {
                vertex[j] += 8;
            }
            else if (vertex[j] >= 8) {
                vertex[j] -= 8;
            }
        }

        // i번째 점의 j번째 선에 연결된 점의 번호 저장
        for (j = 0; j < edgeCnt; j++) {
            vertex_list[i][j] = vertex[j];
        }

        //점 개수 다 세고, 다음 점까지 이동하는 코드
        Left_Forward();
        Right_Forward();
        Move(3000, 3000);
        Clock_Delay1ms(50);

        Rotate_CounterClock_p1(1); //일단 다음 선까지 돌고
        while(status != ndot) {
            //다음 점이 나올때까지 find
            find(); 
        }
    }

    // Phase1 종료 후 후진으로 startline까지 돌아옴
    Left_Forward();
    Right_Forward();
    Move(3000, 3000);
    Clock_Delay1ms(100);

    Move(0,0);
    Clock_Delay1ms(100);

    Left_rotate_135_degrees(); //시작 점에서 왼쪽으로 세번 돌아 직선 맞추기
    Move(0,0);
    Clock_Delay1ms(100);

    while(status != nfinishline) {
        reverse_find(); //후진해서 도착 지점으로
    }

    /* ========== 그래프 저장 및 Euler path 탐색 ========== */
    // 그래프 초기화
    int j;
    for(i = 0; i < 8; i++) {
        for(j = 0; j < 8; j++) {
            graph[i][j] = -1;
        }
    }

    //그래프 생성
    for(i = 0; i < 8; i++){
        int j;
        int len = vertex_length[i];

        for(j = 0; j < len; j++) {
            if(j == len-1) {
                graph[i][vertex_list[i][0]] = j;
            } else {
                graph[i][vertex_list[i][j+1]] = j;
            }
        }
    }

    //그래프를 사용해 경로 생성
    FindEuler(0);

    Clock_Delay1ms(2000);

    /* ========== Phase 2 ========== */
    // 첫 점을 찾을 때까지 find를 돌려 이동하게 한다.
    while(status!=ndot) {
        find();
    }

    Move(3000, 3000);
    Clock_Delay1ms(180);
    Right_rotate_135_degrees_p2(); //135도 회전한다.

    // 한 붓 그리기 경로 시작
    int eruler = 15;
    while(eruler!=-1){
        int nextline = Euler_path[eruler][1];

        if(nextline == 1) {
            Rotate_Clock(nextline+1);
        }
        else if(nextline == 2){
            Rotate_CounterClock(nextline+2);
        }
        else if(nextline == 3){
            Rotate_CounterClock(1);
        }
        eruler--;

        while(status != ndot){
            find();
        }
    }

    // 한 붓 그리기 종료(7->0) 후 startline으로 복귀
    Rotate_Clock(1);
    while(status != nstartline) {
        find();
    }
}

// DFS
int dfsCnt=0;
void FindEuler(int vertex){
    int edge_num = -1;

    int i;
    for(i = 0; i < 8; i++){
        if(graph[vertex][i] != -1){
            edge_num = graph[vertex][i];
            int rev_edge_num = graph[i][vertex];

            graph[vertex][i] = -1;
            graph[i][vertex] = -1;

            // 다음 점에서 갈 수 있는 선이 있는지 확인
            int flag=0;
            int j;
            for(j = 0; j < 8; j++) {
                if(graph[i][j] != -1) {
                    break;
                }
                if(j == 7) {
                    flag = 1; // 갈 수 있는 선이 없다면 flag
                }
            }

            if(flag) {
                if(dfsCnt == 15) {
                    // 모든 선을 탐색했다면 반복문을 끝내고 재귀를 돌며 Euler_path 저장
                    break;
                }
                // 모든 선을 탐색하지 않았는데 다음 점에서의 경로가 없다면 edge_num을 rollback하고, for문 첫번째로 되돌아감
                graph[vertex][i] = edge_num;
                graph[i][vertex] = rev_edge_num;
                
                continue;
            }

            dfsCnt++;
            FindEuler(i);
        }
    }

    Euler_path[path_num][0] = vertex;
    Euler_path[path_num][1] = edge_num;
    path_num++;
}

// 각 점에 연결되어있는 선 개수 확인 (Phase 1에서 사용)
int Count_edge(int index) {
    int cnt = 0;
    int prev_count = 0;

    int i;
    int rotate_value = 660;
    if(index == 0) { rotate_value = 710 };

    for(i = 0; i < rotate_value; i++) { //돌면서 개수 세기 시작
      int on_line;
      int on_blank;

        Clock_Delay1ms(5);
        Left_Forward();
        Right_Backward();
        Move(2300,2300); //소폭 수정
        Clock_Delay1us(100); 

        if(i < 200) { continue; }

        Turn_on_IR();   
        on_line = P7->IN & 0x18;
        on_blank = P7->IN & 0x10;

        if(prev_count==0 && on_line == 0x18) {
            cnt++;
            prev_count++;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x01;
        }   
        else if(prev_count==1 && on_blank == 0){
            prev_count--;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x04;
        }   

        // Turn off IR LEDs
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

    }

    P2->OUT &= ~0x07;
    P2->OUT |= 0x02;
    Move(0,0);
    
    return cnt;
}

void find() {
    Turn_on_IR();
    int on_startline;
    int on_dot;
    int on_line;
    int out_line_left;
    int out_line_right;

    prev_status = status; //이전 state 업데이트

    on_startline = P7->IN & 0xFF;
    on_line = P7->IN & 0x18;
    on_dot = P7->IN & 0x3C;
    out_line_left = P7->IN & 0x08;
    out_line_right = P7->IN & 0x10;

    if(on_startline == 0xFF){
        if(prev_status == nline) { line(); }
        else { startline(); }
    }
    else if((on_dot == 0x3C) && ((P7->IN & 0xC3) == 0) && (prev_status != ndot) && (dot_detect_counter == 0)) { dot(); }
    else if(on_dot == 0x3C && dot_detect_counter!=0){ line(); }
    else if(on_line == 0x18){ line(); }
    else if(out_line_left == 0x08) {
        // turn right
        Left_Forward();
        Right_Backward();
        Move(2500,1250);
        Clock_Delay1us(1);
    }
    else if(out_line_right == 0x10) {
        // turn left
        Left_Backward();
        Right_Forward();
        Move(1250,2500);
        Clock_Delay1us(1);
    }
    else {
        // blank or not match
        Move(0,0);
    }

    // Turn off IR LEDs
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    Clock_Delay1ms(5);

    P2->OUT &= ~0x07;
}

void startline()
{
    printf("startline\n");
    status = nstartline;

    P2->OUT |= 0x01; // red
    // Move Forward
    Left_Forward();
    Right_Forward();
    Move(4000, 4000);
    Clock_Delay1ms(10);
}

void dot() {
    printf("dot\n");
    status = ndot;
    dot_detect_counter = 1;

    P2->OUT |= 0x02; // green
    Move(0,0);
    Clock_Delay1ms(100);
}

void line() {
    printf("line\n");
    status = nline;

    P2->OUT |= 0x04; //blue
    // Move Forward
    Left_Forward();
    Right_Forward();
    Move(3400,3400);
}

// 선을 따라 후진
void reverse_find() {
    Turn_on_IR();
    int on_startline;
    int on_line;
    int out_line_left;
    int out_line_right;

    prev_status = status; //이전 state 업데이트

    on_startline = P7->IN & 0xFF;
    on_line = P7->IN & 0x18;
    out_line_left = P7->IN & 0x08;
    out_line_right = P7->IN & 0x10;

    if(on_startline == 0xFF) {
        status = nfinishline;
        P2->OUT |= 0x01; // red
        Move(0,0);
    }
    else if(on_line == 0x18) {
        Back_line();
    }
    else if(out_line_left == 0x08) {
        // turn right
        Left_Forward();
        Right_Backward();
        Move(2800,1400);
        Clock_Delay1us(1);
    }
    else if(out_line_right == 0x10) {
        // turn left
        Left_Backward();
        Right_Forward();
        Move(1400,2800);
        Clock_Delay1us(1);
    }
    else {
        // blank or not match
        Move(0,0);
    }

    // Turn off IR LEDs
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;

    Clock_Delay1ms(5);

    P2->OUT &= ~0x07;
}

void Back_line() {
    status = nline;
    P2->OUT |= 0x04; //blue
    // Move Backward
    Left_Backward();
    Right_Backward();
    Move(3400,3400);
}

void Left_rotate_45_degrees() {
    Left_Backward();
    Right_Forward();
    Move(3700,3700);

    Clock_Delay1ms(TURN_TIME_MS/2);

    Move(0, 0);
}

void Right_rotate_45_degrees() {
    Left_Forward();
    Right_Backward();
    Move(3000,3000);
    Clock_Delay1ms(TURN_TIME_MS/2);

    Move(0, 0);
}

void Left_rotate_90_degrees() {
    Left_Backward();
    Right_Forward();
    Move(3000,4000);

    Clock_Delay1ms(TURN_TIME_MS);

    Move(0,0);
}

void Right_rotate_90_degrees() {
    Left_Forward();
    Right_Backward();
    Move(3000,3000);

    Clock_Delay1ms(TURN_TIME_MS*1.4);

    Move(0, 0);
}

void Left_rotate_135_degrees() {
    Left_Backward();
    Right_Forward();
    Move(3500,3500);

    Clock_Delay1ms(TURN_TIME_MS * 1.9);

    Move(0, 0);
}

void Right_rotate_135_degrees() {
    Left_Forward();
    Right_Backward();
    Move(3500,3500);

    Clock_Delay1ms(TURN_TIME_MS * 1.7);

    Move(0, 0);
}

void Right_rotate_135_degrees_p2() {
    Left_Forward();
    Right_Backward();
    Move(3500,3500);

    Clock_Delay1ms(TURN_TIME_MS * 1.5);

    Move(0, 0);
}

void Rotate_Clock(int targetLine) {
    int cnt = 0;
    int prev_count = 0;

    if(targetLine == 1) {
        Move(3200, 3200);
        Clock_Delay1ms(200);
    }
    else if(targetLine == 2) {
        Move(3000, 3000);
        Clock_Delay1ms(170);
    }

    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    Clock_Delay1ms(5);

    while(1){
        Move(3000, 3000);
        Clock_Delay1ms(1);

        Turn_on_IR();
        int line_check = P7->IN & 0x18;
        if(line_check != 0x18) { break; }
    }

    while(1) {
        int on_line;
        int on_blank;

        Clock_Delay1ms(5);
        Left_Forward();
        Right_Backward();
        Move(3500,3500);
        Clock_Delay1us(100);

        Turn_on_IR();

        on_line = P7->IN & 0x18;
        on_blank = P7->IN & 0x10;

        if(prev_count == 0 && on_line == 0x18) {
            cnt++;
            prev_count++;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x01;
        }
        else if(prev_count == 1 && on_blank == 0){
            prev_count--;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x04;
        }

        if (cnt == targetLine) {
            Move(0,0);
            Clock_Delay1ms(100);
            P2->OUT &= ~0x07;
            status = nline;
            break;
        }

        // Turn off IR LEDs
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

    }

    P2->OUT &= ~0x07;
    P2->OUT |= 0x02;
    TimerA2_Init(&task,DOT_DETECT_DELAY);
}

void Rotate_CounterClock(int targetLine) {
    int cnt = 0;
    int prev_count = 0;

    if(targetLine == 1){
        Left_Forward();
        Right_Forward();
        Move(3000, 3000);
        Clock_Delay1ms(230);
    }
    else if(targetLine == 2){
        Left_Forward();
        Right_Forward();
        Move(3000, 3000);
        Clock_Delay1ms(180);
    }

    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    Clock_Delay1ms(5);

    while(1){
        Move(3000, 3000);
        Clock_Delay1ms(1);

        Turn_on_IR();
        int line_check = P7->IN & 0x18;
        if(line_check != 0x18) { break; }
    }

    while(1) {
        int on_line;
        int on_blank;

        Clock_Delay1ms(5);
        Left_Backward();
        Right_Forward();
        Move(3500,3500);
        Clock_Delay1us(100);

        Turn_on_IR();

        on_line = P7->IN & 0x18;
        on_blank = P7->IN & 0x10;

        if(prev_count == 0 && on_line == 0x18) {
            cnt++;
            prev_count++;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x01;
        }
        else if(prev_count == 1 && on_blank == 0){
            prev_count--;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x04;
        }

        if (cnt == targetLine) {
            Move(0,0);
            Clock_Delay1ms(100);
            P2->OUT &= ~0x07;
            status = nline;
            break;
        }

        // Turn off IR LEDs
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

    }

    P2->OUT &= ~0x07;
    P2->OUT |= 0x02;
    TimerA2_Init(&task,DOT_DETECT_DELAY);
}

void Rotate_CounterClock_p1(int targetLine) {
    int cnt = 0;
    int prev_count = 0;

    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    Clock_Delay1ms(5);

    while(1) {
        Move(3000, 3000);
        Clock_Delay1ms(1);

        Turn_on_IR();
        int line_check = P7->IN & 0x18;

        if(line_check != 0x18) { break; }
    }

    while(1) {
        int on_line;
        int on_blank;

        Clock_Delay1ms(5);
        Left_Backward();
        Right_Forward();
        Move(3000,3000);
        Clock_Delay1us(100);

        Turn_on_IR();

        on_line = P7->IN & 0x18;
        on_blank = P7->IN & 0x10;

        if(prev_count == 0 && on_line == 0x18) {
            cnt++;
            prev_count++;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x01;
        }
        else if(prev_count == 1 && on_blank == 0){
            prev_count--;
            P2->OUT &= ~0x07;
            P2->OUT |= 0x04;
        }

        if (cnt == targetLine) {
            Move(0,0);
            Clock_Delay1ms(100);
            P2->OUT &= ~0x07;
            status = nline;
            break;
        }

        // Turn off IR LEDs
        P5->OUT &= ~0x08;
        P9->OUT &= ~0x04;

    }
    P2->OUT &= ~0x07;
    P2->OUT |= 0x02;
    TimerA2_Init(&task,DOT_DETECT_DELAY);
}

void Move(uint16_t leftDuty, uint16_t rightDuty) {
    P3->OUT |= 0xC0;
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

void Left_Forward() {
    P5->OUT &= ~0x10;
}

void Left_Backward() {
    P5->OUT |= 0x10;
}

void Right_Forward() {
    P5->OUT &= ~0x20;
}

void Right_Backward() {
    P5->OUT |= 0x20;
}

void (*TimerA2Task) (void);
void TimerA2_Init (void(*task) (void), uint16_t period) {
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00) | 0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();

}

void task(void){
        dot_detect_counter = 0;
}

void PWM_Init34(uint16_t period, uint16_t duty3,uint16_t duty4){
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;

    TIMER_A0->CCTL[0] = 0x0080;
    TIMER_A0->CCR[0] = period;

    TIMER_A0->EX0 = 0x0000;

    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    TIMER_A0->CTL = 0x02F0;
}

void LED_Init(void) {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}

void IR_Init(void) {
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= 0xFF;
}

void Motor_Init(void){
    P3->SEL0 &= ~0x0C0;
    P3->SEL1 &= ~0x0C0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    P5->SEL0 &= ~0x030;
    P5->SEL1 &= ~0x030;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    P2->SEL0 &= ~0x0C0;
    P2->SEL1 &= ~0x0C0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    PWM_Init34(15000, 0, 0);
}

void PWM_Duty3(uint16_t duty3) {
    TIMER_A0->CCR[3] = duty3;
}

void PWM_Duty4(uint16_t duty4) {
    TIMER_A0->CCR[4] = duty4;
}

void Turn_on_IR() {
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;

    P7->DIR = 0xFF;
    P7->OUT = 0xFF;
    Clock_Delay1us(10);

    P7->DIR = 0x00;

    Clock_Delay1us(1000);
}

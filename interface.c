#include "API.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

int x = 0;
int y = 0;
int change = 0; // 0 = up , 1 = left, 2 = down, 3 = right
int stack[100][2];
int stack_index = 0;
int first_turn[2] = {NULL, NULL};
int tmp_length = 0;

// These directions used instead of "F" "R" "L" respectively to work with array of integers instead of list in PYTHON
int forward_direction = 1, right_direction = 2, left_direction = 3;

int potential_value[8][8] = {
        {6, 5, 4, 3, 3, 4, 5, 6},
        {5, 4, 3, 2, 2, 3, 4, 5},
        {4, 3, 2, 1, 1, 2, 3, 4},
        {3, 2, 1, 0, 0, 1, 2, 3},
        {3, 2, 1, 0, 0, 1, 2, 3},
        {4, 3, 2, 1, 1, 2, 3, 4},
        {5, 4, 3, 2, 2, 3, 4, 5},
        {6, 5, 4, 3, 3, 4, 5, 6}
};

int new_potential_value[8][8] = {
        {0, 0, 0, 0,   0,   0, 0, 0},
        {0, 0, 0, 0,   0,   0, 0, 0},
        {0, 0, 0, 0,   0,   0, 0, 0},
        {0, 0, 0, 255, 255, 0, 0, 0},
        {0, 0, 0, 255, 255, 0, 0, 0},
        {0, 0, 0, 0,   0,   0, 0, 0},
        {0, 0, 0, 0,   0,   0, 0, 0},
        {0, 0, 0, 0,   0,   0, 0, 0}
};

int goal_index[4][2] = {{3, 3},
                        {3, 4},
                        {4, 3},
                        {4, 4}};

int visited[16][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int marked[16][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int visit_count[16][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int in_stack[16][16] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
int last3[4] = {NULL, NULL, NULL, NULL};
int last_potiential = 0;
int last_on_path_potiental = 0;
int home_time = 0;
int stop_steps = 150;

void log(char *text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int get_number_of_ways(char ways[]);
void check_around_index(char way, int index[]);
void turn_left();
void make_move(char direction, int mode);
int dead_end_route(int mode);
int check_next_marked(char way);
void reached_goal();
void move_step(int mode);
int check_next_visited(char way);
int check_next_potiential(char way);
int check_next_visited_count(char way);
void go_home(int tmp_stack[][2], int length);
void follow_stack();
int check_next_in_stack(char way);
char check_if_around_in_stack();
void move_to( int next_x, int next_y);
void shortest_path();
int check_next_new_potiential(char way);
void move_step(int mode);
void mark();
void mark2();


int main() {
    log("Running...");
    API_setColor(0, 0, 'G');
    API_setText(x, y, "index");
    int iii = 0;
    while (iii++ < 9999) {
        iii++;
        bool inGoal_index = false;
        int arrTemp[2];
        arrTemp[0] = x;
        arrTemp[1] = y;
        for (int i = 0; i < 4; ++i)
            if (goal_index[i][0] == x && goal_index[i][1] == y)
                inGoal_index = true;
        if (inGoal_index == true) {
            log("winnn");
            int current_path = stack_index;
            for (int i = 0; i < stack_index; ++i) {
                log("------------------1");
                new_potential_value[stack[i][0]][stack[i][1]] = current_path - i;
                char arr[5];
                sprintf(arr,"%d",current_path - i);
                log(arr);
                API_setText(stack[i][0], stack[i][1], arr);
                log("------------------1.6");
            }
            log("------------------2");
            reached_goal();
            log("------------------3");
        }
        if (!visited[x][y])
            visited[x][y] = 1;
        visit_count[x][y] += 1;

        if (!in_stack[x][y]) {
            stack[stack_index][0] = x;
            stack[stack_index][1] = y;
            ++stack_index;
            in_stack[x][y] = 1;
            API_setColor(x, y, 'G');
        } else {
            while (true) {
                int top_of_stack[2];
                top_of_stack[0] = stack[stack_index - 1][0];
                top_of_stack[1] = stack[stack_index - 1][1];
                if (top_of_stack[0] == x && top_of_stack[1] == y)
                    break;
                else {
                    in_stack[top_of_stack[0]][top_of_stack[1]] = 0;
                    stack[stack_index - 1][0] = 0;
                    stack[stack_index - 1][1] = 0;
                    --stack_index;
                    if (!marked[top_of_stack[0]][top_of_stack[1]])
                        API_setColor(top_of_stack[0], top_of_stack[1], 'B');
                }
            }
        }
        int number_of_ways = 0;
        char ways[4];
        number_of_ways = get_number_of_ways(ways);
        
        if (number_of_ways > 0) {
            for (int i = 0; i < number_of_ways; ++i) {
                int around_index[2];
                check_around_index(ways[i], around_index);
                if (in_stack[around_index[0]][around_index[1]]) {
                    while (true) {
                        int top_of_stack[2];
                        top_of_stack[0] = stack[stack_index - 1][0];
                        top_of_stack[1] = stack[stack_index - 1][1];
                        if (top_of_stack[0] == around_index[0] && top_of_stack[1] == around_index[1]) {
                            stack[stack_index][0] = x;
                            stack[stack_index][1] = y;
                            ++stack_index;
                            API_setColor(x, y, 'G');
                            break;
                        } else {
                            in_stack[top_of_stack[0]][top_of_stack[1]] = 0;
                            stack[stack_index - 1][0] = 0;
                            stack[stack_index - 1][1] = 0;
                            --stack_index;
                            if (!marked[top_of_stack[0]][top_of_stack[1]])
                                API_setColor(top_of_stack[0], top_of_stack[1], 'B');
                        }
                    }
                }
            }
        }
        if (number_of_ways == 0){
            log("0 ways");
            if (x==0 && y==0){
                turn_left();
                turn_left();
                make_move('F', 0);
            }
            else
                dead_end_route(0);
        }
        else if( number_of_ways == 1){
            log("1 ways which is " + ways[0]);
            if (check_next_marked(ways[0])){
                turn_left();
                turn_left();
                move_step(0);
            }
            else
                make_move(ways[0], 0);
        }
        else{
            log("ways more than 1, which are ");
           // log(ways);
            //log(ways[1]);
           
            for (int i=0;i<number_of_ways;i++){
                if (check_next_marked(ways[i])){
                    log("9999999999999999999");
                    if(i==number_of_ways-1){
                        number_of_ways--;

                    }
                    else{
                        number_of_ways--;
                        for(int j=i;j<number_of_ways;++j){
                            ways[j]=ways[j+1];
                        }

                    }
                }
            }
            int tmp_way_count;
            int tmp_way_count_index =0;
            int tmp_way_count3;
            char tmp_way_count2;
            int min_value=255;
            for (int i=0;i<number_of_ways;++i){
                log("-0000000000000000000000000000000000000000000000");
                //log(ways[i]);
                if  (!check_next_visited(ways[i])){
                    
                    tmp_way_count=check_next_potiential(ways[i]);
                    char arr[5];
                    sprintf(arr,"%d",tmp_way_count);
                    log("-0000000000000000000000000000000000000000000000");
                    //log(arr);
                    if(tmp_way_count<min_value){
                        min_value =tmp_way_count;
                        tmp_way_count2=ways[i];
                    }
                    ++tmp_way_count_index;
                }
            }
            if (tmp_way_count_index > 0){
                make_move(tmp_way_count2, 0);
                continue;
            }
            tmp_way_count_index=0;
            for (int i=0;i<number_of_ways;++i){
                tmp_way_count=check_next_visited_count(ways[i]);
                tmp_way_count2=ways[i];
                tmp_way_count3=check_next_potiential(ways[i]);

            }
            
            if (tmp_way_count < 4)
                make_move(tmp_way_count2, 0);
            else{
                make_move(tmp_way_count2, 0);
            }
        }
    }
}

void reached_goal(){
    API_clearAllText();
    int current_path = stack_index;
    int i = 0;
    for(int j=0;j<stack_index;++j){
        new_potential_value[stack[j][0]][stack[j][1]] = current_path - i;
        char arr[5];
        sprintf(arr,"%d",current_path - i);
        API_setText(stack[j][0], stack[j][1],arr);
        i += 1;
    }
    int number_of_ways=0;
    char ways [4];
    number_of_ways = get_number_of_ways(ways);
    for(int i=0;i<4;i++){
        number_of_ways = get_number_of_ways(ways);
        int mode=1;
        make_move(ways[0], mode);
    }
    number_of_ways = get_number_of_ways(ways);
    int around_index[2];
    check_around_index(ways[0], around_index);
    int in_flag = 0;
    for(int i=0;i<4;++i){
        if(around_index[0] == goal_index[i][0] && around_index[1] == goal_index[i][1])
            in_flag=1;
    }
    int mode=1;
    if (in_flag==1)
        make_move(ways[1], mode);
    else
        make_move(ways[0], mode);

    last_potiential = new_potential_value[x][y];
    for(int i=0;i<4;++i){
        visited[goal_index[i][0]][goal_index[i][1]] = 1;
    }

    int tmp_stack[100][2];
    int tmp_stack_index=0;
    int last_in_stack_node [100];
    i = 0;

    while(true){

        if (i == stop_steps){
            go_home(tmp_stack,tmp_stack_index);
        }
        i += 1;

        if (!visited[x][y])
            visited[x][y] = 1;
        visit_count[x][y] += 1;

        last_potiential = new_potential_value[x][y];

        number_of_ways = get_number_of_ways(ways);
        if (number_of_ways > 0){
            for(int i=0;i<number_of_ways;++i){
                int neighbour[2];
                check_around_index(ways[i], neighbour);
                if  (!new_potential_value[neighbour[0]][neighbour[1]]){
                    if (!marked[neighbour[0]][neighbour[1]]){
                        log("frommmmm1  : last = ");
                        new_potential_value[neighbour[0]][neighbour[1]] = last_potiential + 1;
                        char arr[5];
                        sprintf(arr,"%d", last_potiential + 1);
                        API_setText(neighbour[0], neighbour[1], arr);
                    }
                }
                else{
                    // check the difference between them
                    if (new_potential_value[neighbour[0]][neighbour[1]] >last_potiential + 1){
                        if (!marked[neighbour[0]][neighbour[1]]){
                            new_potential_value[neighbour[0]][neighbour[1]] =last_potiential + 1;
                            char arr[5];
                            sprintf(arr,"%d", last_potiential + 1);
                            API_setText(neighbour[0], neighbour[1], arr);
                        }
                    }
                }
            }
        }
        if (true){
            if (in_stack[x][y]){

                last_on_path_potiental = new_potential_value[x][y];
                tmp_stack_index=0;
                tmp_stack[tmp_stack_index][0]=x;
                tmp_stack[tmp_stack_index][1]=y;
                tmp_stack_index++;
            }
            else{
                if (!marked[x][y]){
                    tmp_stack[tmp_stack_index][0]=x;
                    tmp_stack[tmp_stack_index][1]=y;
                    tmp_stack_index++;
                }
            }
            last_potiential = new_potential_value[x][y];
        }
        number_of_ways = get_number_of_ways(ways);

        if (number_of_ways == 0){
            log("0 ways");
            if (x==0 &&  y==0){
                turn_left();
                turn_left();
                make_move('F', 1);
            }
            else
                dead_end_route(1);
        }
        else if (number_of_ways == 1){
            log("1 ways which is " + ways[0]);
            if (check_next_marked(ways[0])){
                turn_left();
                turn_left();
                move_step(1);
            }
            else
                make_move(ways[0], 1);
        }
        else{
            log("ways more than 1, which are ");
            for(int i=0;i<number_of_ways;++i){
                if (check_next_marked(ways[i])){
                    if(i==number_of_ways-1){
                        number_of_ways--;

                    }
                    else{
                        number_of_ways--;
                        for(int j=i;j<number_of_ways;++j){
                            ways[j]=ways[j+1];
                        }

                    }
                }

            }
            int tmp_way_count[100][2];
            int tmp_way_count_index=0;
            int tmp_way_count1=0;
            char tmp_way_count2;
            int min_value=255;
            for(int i=0;i<number_of_ways;++i){
                if (!check_next_visited(ways[i])){
                    int tmp = check_next_visited_count(ways[i]);
                    if(tmp<min_value){
                         tmp_way_count1=tmp;
                         min_value=tmp;
                        tmp_way_count2=ways[i];
                    }
                    ++tmp_way_count_index;
                }
            }
            if(tmp_way_count_index>0){
                make_move(tmp_way_count2, 1);
                continue;
            }
            tmp_way_count_index=0;
            for(int i=0;i<number_of_ways;++i){
                    int tmp = check_next_visited_count(ways[i]);
                    if(tmp<min_value){
                         tmp_way_count1=tmp;
                         min_value=tmp;
                        tmp_way_count2=ways[i];
                    }
                  ++tmp_way_count_index;
                
            }

            make_move(tmp_way_count2, 1);
        }
    }
}
void go_home(int tmp_stack[][2], int length){
    log("hoooooooooooooooooooooooome");
    while (true){

        if (in_stack[x][y])
            follow_stack();

        log("moving ----------");
        for(int j=length-1 ; j>=0 ;j--){
            //for i in range(len(tmp_stack)-1, -1, -1){
            int node[2];
            node[0] = tmp_stack[j][0];
            node[1] = tmp_stack[j][1];
             char arr[5];
            sprintf(arr,"%d",node[0]);
        log("3333333333333");
        log(arr);
        sprintf(arr,"%d",node[1]);
        log("3333333333333333");
        log(arr);
         
         
            char around_in_stack = check_if_around_in_stack();
            log("around in stack :");
            if (around_in_stack != 'X'){
                make_move(around_in_stack, 1);
                break;
            }
            if (!(marked[node[0]][node[1]]) && (x != node[0] || y != node[1]))
                move_to(node[0], node[1]);
        }
    }
}
char check_if_around_in_stack(){
    int number_of_ways=0;
    char ways [4];
    number_of_ways = get_number_of_ways(ways);
    if (number_of_ways < 2)
        return 'X';
    else{
        for (int i=0;i<number_of_ways;++i){
            if (check_next_in_stack(ways[i]))
                return ways[i];
        }
    }
}
void follow_stack(){

    int index_in_stack = 0;
    for (int i=0;i<stack_index;++i){
        if (stack[i][0] == x && stack[i][1]==y )
            break;
        index_in_stack += 1;
    }
    for(int i=index_in_stack-1;i>=0;--i){
        log(stack[i]);
        int node[2];
        node[0] = stack[i][0];
        node[1] = stack[i][1];
        char arr[5];
        sprintf(arr,"%d",node[0]);
        log("555555555");
        log(arr);
         
        sprintf(arr,"%d",node[1]);
        log(arr);
        move_to(node[0], node[1]);
    }
    shortest_path();
}
void shortest_path(){
    log("---------------------------------------------------------------");

    API_clearAllText();
    API_clearAllColor();

    turn_left();
    turn_left();
    int i = 0;

    while (true){
        for(int j=0;j<4;++j)
            if  (x==goal_index[j][0] && y==goal_index[j][1]){
                log("We Have A Winner");
                exit(0);
            }
            else{
                API_setColor(x, y, 'Y');
                 char arr[5];
                sprintf(arr,"%d",i);
                API_setText(x, y,arr);
                i += 1;
                int number_of_ways=0;
                char ways [4];
                char best_way;
                number_of_ways = get_number_of_ways(ways);

                log(",   ways are = ");
                int least_num = 255;
                int first_value = check_next_new_potiential(ways[0]);
                if (first_value){
                    least_num =first_value;
                    best_way = ways[0];
                }
                for(int k=0;k<number_of_ways;++k){
                    log("way: " + ways[k]);
                    log( ",   with pot = ");
                    int value = check_next_new_potiential(ways[k]);
                    if (value > 0 && value <= least_num){
                        least_num = value;
                        best_way = ways[k];
                    }
                }
                make_move(best_way, 0);
            }
    }
}
void move_to( int next_x, int next_y){
    int mode;
    if (change == 0){
        if (next_x > x)
            make_move('R', 1);
        else if (next_x < x)
            make_move('L', 1);
        else if (next_y > y)
            make_move('F', 1);
        else{
            turn_left();
            turn_left();
            move_step(1);
        }
    }
    else if (change == 1){
        if (next_x > x){
            turn_left();
            turn_left();
            move_step(1);
        }
        else if (next_x < x)
            make_move('F', 1);
        else if (next_y > y)
            make_move('R', 1);
        else
            make_move('L', 1);
    }
    else if (change == 2){
        if (next_x > x)
            make_move('L', 1);
        else if (next_x < x)
            make_move('R', 1);
        else if (next_y > y){
            turn_left();
            turn_left();
            move_step(1);
        }
        else
            make_move('F', 1);
    }
    else if (change == 3){
        if (next_x > x)
            make_move('F', 1);
        else if (next_x < x){
            turn_left();
            turn_left();
            move_step(1);
        }
        else if (next_y > y)
            make_move('L', 1);
        else
            make_move('R', 1);
    }

}

void check_around_index(char way, int index[]){

    if (change == 0){
        if (way == 'F'){
            index[0] = x;
            index[1] = y+1;
        }
        else if( way == 'R'){
            index[0] = x + 1;
            index[1] = y;
        }
        else if( way == 'L'){
            index[0] = x - 1;
            index[1] = y;
        }
    }

    if (change == 1){
        if (way == 'F'){
            index[0] = x - 1;
            index[1] = y;
        }
        else if( way == 'R'){
            index[0] = x;
            index[1] = y + 1;
        }
        else if( way == 'L'){
            index[0] = x;
            index[1] = y - 1;
        }
    }

    if (change == 2){
        if (way == 'F'){
            index[0] = x;
            index[1] = y - 1;
        }
        else if( way == 'R'){
            index[0] = x - 1;
            index[1] = y;
        }
        else if( way == 'L'){
            index[0] = x + 1;
            index[1] = y;
        }
    }

    if (change == 3){
        if (way == 'F'){
            index[0] = x + 1;
            index[1] = y;
        }
        else if( way == 'R'){
            index[0] = x;
            index[1] = y - 1;
        }
        else if( way == 'L'){
            index[0] = x;
            index[1] = y + 1;
        }
    }
   }


int check_next_potiential(char way){

if (change == 0){

    if (way == 'F')
        return potential_value[x][y + 1];
    else if( way == 'R')
        return potential_value[x + 1][y];
    else if( way == 'L')
        return potential_value[x - 1][y];
}

else if( change == 1){

    if (way == 'F')
        return potential_value[x - 1][y];
    else if( way == 'R')
        return potential_value[x][y + 1];
    else if( way == 'L')
        return potential_value[x][y - 1];
}

else if( change == 2){

    if (way == 'F')
        return potential_value[x][y - 1];
    else if( way == 'R')
        return potential_value[x - 1][y];
    else if( way == 'L')
        return potential_value[x + 1][y];

}
else if( change == 3){

    if (way == 'F')
        return potential_value[x + 1][y];
    else if( way == 'R')
        return potential_value[x][y - 1];
    else if( way == 'L')
        return potential_value[x][y + 1];
}
}


int check_next_new_potiential(char way){

    if (change == 0){
        if (way == 'F')
            return new_potential_value[x][y + 1];
        else if( way == 'R')
            return new_potential_value[x + 1][y];
        else if( way == 'L')
            return new_potential_value[x - 1][y];
    }


    else if( change == 1){
        if (way == 'F')
        return new_potential_value[x - 1][y];
        else if( way == 'R')
            return new_potential_value[x][y + 1];
        else if( way == 'L')
            return new_potential_value[x][y - 1];
    }


    else if( change == 2){
        if (way == 'F')
            return new_potential_value[x][y - 1];
        else if( way == 'R')
            return new_potential_value[x - 1][y];
        else if( way == 'L')
            return new_potential_value[x + 1][y];
    }


    else if( change == 3){
        if (way == 'F')
        return new_potential_value[x + 1][y];
        else if( way == 'R')
            return new_potential_value[x][y - 1];
        else if( way == 'L')
            return new_potential_value[x][y + 1];
    }

}



int check_next_visited(char way){

    if (change == 0){

        if (way == 'F')
            return visited[x][y + 1];
        else if( way == 'R')
            return visited[x + 1][y];
        else if( way == 'L')
            return visited[x - 1][y];
    }

    else if( change == 1){
        if (way == 'F')
            return visited[x - 1][y];
        else if( way == 'R')
            return visited[x][y + 1];
        else if( way == 'L')
            return visited[x][y - 1];
    }


    else if( change == 2){
        if (way == 'F')
            return visited[x][y - 1];
        else if( way == 'R')
            return visited[x - 1][y];
        else if( way == 'L')
            return visited[x + 1][y];
    }


    else if( change == 3){
        if (way == 'F')
            return visited[x + 1][y];
        else if( way == 'R')
            return visited[x][y - 1];
        else if( way == 'L')
            return visited[x][y + 1];
    }

}



int check_next_in_stack(char way) {

    if (change == 0){
        if (way == 'F')
            return in_stack[x][y + 1];
        else if( way == 'R')
            return in_stack[x + 1][y];
        else if( way == 'L')
            return in_stack[x - 1][y];

    }

    else if( change == 1){
        if (way == 'F')
            return in_stack[x - 1][y];
        else if( way == 'R')
            return in_stack[x][y + 1];
        else if( way == 'L')
            return in_stack[x][y - 1];
    }


    else if( change == 2){
        if (way == 'F')
            return in_stack[x][y - 1];
        else if( way == 'R')
            return in_stack[x - 1][y];
        else if( way == 'L')
            return in_stack[x + 1][y];
    }


    else if( change == 3){
        if (way == 'F')
            return in_stack[x + 1][y];
        else if( way == 'R')
            return in_stack[x][y - 1];
        else if( way == 'L')
            return in_stack[x][y + 1];
    }

}



int check_next_visited_count(char way){

    if (change == 0){
        if (way == 'F')
            return visit_count[x][y + 1];

        else if( way == 'R')
            return visit_count[x + 1][y];

        else if( way == 'L')
            return visit_count[x - 1][y];

    }


    else if( change == 1){

        if (way == 'F')
            return visit_count[x - 1][y];

        else if( way == 'R')
            return visit_count[x][y + 1];

        else if( way == 'L')
            return visit_count[x][y - 1];

    }


    else if( change == 2){
        if (way == 'F')
            return visit_count[x][y - 1];

        else if( way == 'R')
            return visit_count[x - 1][y];

        else if( way == 'L')
            return visit_count[x + 1][y];

    }


    else if( change == 3){
        if (way == 'F')
            return visit_count[x + 1][y];

        else if( way == 'R')
            return visit_count[x][y - 1];

        else if( way == 'L')
            return visit_count[x][y + 1];

    }

}


int check_next_marked(char way) {

    if (change == 0) {
        if (way == 'F')
            return marked[x][y + 1];

        else if (way == 'R')
            return marked[x + 1][y];

        else if (way == 'L')
            return marked[x - 1][y];


    } else if (change == 1) {
        if (way == 'F')
            return marked[x - 1][y];

        else if (way == 'R')
            return marked[x][y + 1];

        else if (way == 'L')
            return marked[x][y - 1];


    } else if (change == 2) {
        if (way == 'F')
            return marked[x][y - 1];

        else if (way == 'R')
            return marked[x - 1][y];

        else if (way == 'L')
            return marked[x + 1][y];


    } else if (change == 3) {
        if (way == 'F')
            return marked[x + 1][y];

        else if (way == 'R')
            return marked[x][y - 1];

        else if (way == 'L')
            return marked[x][y + 1];

    }

}

// Updated
void turn_right() {
    change = change - 1;
    if (change == -1)
        change = 3;
    API_turnRight();
}

// Updated
void turn_left() {
    change = (change + 1) % 4;
    API_turnLeft();
}

// Updated
void move_step(int mode) {

    API_moveForward();

    if (mode)
        mark2();
    else
        mark();
}

// Updated
int dead_end_route(int mode) {

    marked[x][y] = 1;
    new_potential_value[x][y] = 255;
    API_setColor(x, y, 'R');
    API_setText(x, y, "255");
    turn_right();
    turn_right();
    make_move('F', mode);

    int number_of_ways;
    char ways[4];
    while (true) {

        number_of_ways = get_number_of_ways(ways);
        if (number_of_ways == 1) {
            marked[x][y] = 1;
            new_potential_value[x][y] = 255;
            API_setColor(x, y, 'R');
            API_setText(x, y, "255");
            make_move(ways[0], mode);
        } else
            return -1;
    }

}

// Updated
/*
 * return 0 for FALSE and 1 for TRUE instead of boolean
 */

int check_if_visited(int x, int y) {

    if (visited[x][y])
        return 1;
    else
        return 0;
}


// Updated, TODO: need work on tmp
/*
 *
 * here tmp length defined as global variable and the function return the list only
 * we need linked list for tmp
 */
int get_number_of_ways(char ways[]) {
    int i = 0;
    ways[0] = NULL;
    ways[1] = NULL;
    ways[2] = NULL;
    if (!API_wallFront()) {
        ways[i] = 'F';
        i++;
    }
    if (!API_wallRight()){
        ways[i] = 'R';
        i++;
    }
    if (!API_wallLeft()){
        ways[i] = 'L';
        i++;
    }
    return i;
}

//Updated

void make_move(char direction, int mode) {

    if (direction == 'F')
        move_step(mode);

    else if (direction == 'L') {
        turn_left();
        move_step(mode);

    } else if (direction == 'R') {
        turn_right();
        move_step(mode);
    }

}

void mark() {

    if (change == 0)
        y += 1;
    else if (change == 1)
        x -= 1;
    else if (change == 2)
        y -= 1;
    else if (change == 3)
        x += 1;
    // API.setColor(x, y, 'B')

    //Convert x & y to string

    char x_result[10];
    char y_result[10];
    char result[25];

    sprintf(x_result, "%d", x);
    sprintf(y_result, "%d", y);
    strcat(result, x_result);
    strcat(result, ",");
    strcat(result, y_result);

    API_setText(x, y, result);

}


// Updated

void mark2() {
    // API.setText(x, y, new_potential_value[x][y])
    if (change == 0) {
        y += 1;
    } else if (change == 1) {
        x -= 1;
    } else if (change == 2) {
        y -= 1;
    } else if (change == 3) {
        x += 1;
    }


}


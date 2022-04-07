#include <iostream>
#include <cstring>
#include <cmath>
#include <queue>
#include <stack>
#include <iomanip>
#include <fstream>
#include <Windows.h>

using namespace std;

typedef int State[9];

const int STATE = 362800 + 5;

State state[STATE], goal;

const int FAC[] = {1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880};

int cantor_hash(const State &s) {
    int res = 0;
    for (int i = 0; i < 9; ++i) {
        int smaller = 0;
        for (int j = i + 1; j < 9; ++j)
            smaller += (s[j] < s[i]);
        res += FAC[8 - i] * smaller;
    }
    return res;
}

const int MISPLACE_SQUARE = 1;
const int SUM_EUCLEDIAN_DISTANCE_SQUARE = 2;
const int SUM_MANHATTAN_DISTANCE_SQUARE = 3;
const int SUM_ROW_COL_SQUARE = 4;
//non admissable
const int NILSSON_SEQUENCE_SCORE = 5;

int misplace_square(const State &now) {
    int price = 0;
    for (int i = 0; i < 9; ++i) {
        if (now[i] != goal[i] && now[i] != 0)
            ++price;
    }
    return price;
}

int sum_eucledian_distance_square(const State &now) {
    int price = 0;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            if (now[i] == goal[j] && i != j && now[i] != 0) {
                int x = (i / 3 - j / 3), y = (i % 3 - j % 3);
                price += x * x + y * y;
            }
        }
    }
    return price;
}

int sum_manhattan_distance_square(const State &now) {
    int price = 0;

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            if (now[i] == goal[j] && i != j && now[i] != 0) {
                int x = abs(i / 3 - j / 3), y = abs(i % 3 - j % 3);
                price += (x + y);
            }
        }
    }
    return price;
}

int sum_row_col_square(const State &now) {
    int price = 0;

    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            if (now[i] == goal[j] && i != j && now[i] != 0) {
                if (i / 3 != j / 3)
                    ++price;
                if (i % 3 != j % 3)
                    ++price;
            }
        }
    }
    return price;
}

int nilsson_sequence_score(const State &now) {
    int clockwise[] = {0, 1, 2, 5, 8, 7, 6, 4, 0};
    int s = 0;
    int p = sum_manhattan_distance_square(now);

    //center
    if (now[4] != 0)
        ++s;
    //clockwise check
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            if (now[clockwise[i]] != 0) {
                if (goal[clockwise[i]] == now[clockwise[j]]
                    && goal[clockwise[i + 1]] != now[clockwise[j + 1]])
                    s += 2;
            }
        }
    }
    return p + 3 * s;
}

int (*choose_cal_price_fun(int c))(const State &now) {
    switch (c) {
        case MISPLACE_SQUARE:
            return misplace_square;
        case SUM_EUCLEDIAN_DISTANCE_SQUARE:
            return sum_eucledian_distance_square;
        case SUM_MANHATTAN_DISTANCE_SQUARE:
            return sum_manhattan_distance_square;
        case SUM_ROW_COL_SQUARE:
            return sum_row_col_square;
        case NILSSON_SEQUENCE_SCORE:
            return nilsson_sequence_score;
        default:
            cout << "wrong h-function" << endl;
            exit(-1);
    }
}

const int DX[] = {-1, 0, 1, 0};
const int DY[] = {0, -1, 0, 1};

int dis[STATE], parent[STATE], price[STATE], head[STATE], next_edge[STATE];

auto comp_price = [](const int& a, const int& b) { return price[a] > price[b]; };

bool insert(const int& now) {
    int hash = cantor_hash(state[now]), u = head[hash];
    while (u) {
        if (!memcmp(state[u], state[now], sizeof(State)))
            return false;
        u = next_edge[u];
    }
    next_edge[now] = head[hash], head[hash] = now;
    return true;
}

void init() {
    memset(state + 2, 0, sizeof(state) - 2 * sizeof(State));
    memset(dis, 0, sizeof(dis));
    memset(parent, 0, sizeof(parent));
    memset(price, 0, sizeof(price));
    memset(head, 0, sizeof(head));
    memset(next_edge, 0, sizeof(next_edge));
}

int A_star(int (*cal_price)(const State &now)) {
    init();
    ofstream fout("out.txt");
    fout << "digraph EightDigit{" << endl;
    priority_queue<int, vector<int>, decltype(comp_price)> states(comp_price);
    int cnt = 1;
    states.push(cnt);
    while (!states.empty()) {
        int index = states.top();
        states.pop();
        State &s = state[index];
        if (!memcmp(s, goal, sizeof(State)))
            return index;
        int z;
        for (z = 0; z < 9; ++z)
            if (!s[z])
                break;
        int x = z / 3, y = z % 3;
        for (int d = 0; d < 4; ++d) {
            int x_new = x + DX[d], y_new = y + DY[d];
            if (x_new >= 0 && x_new < 3 && y_new >= 0 && y_new < 3) {
                int z_new = x_new * 3 + y_new;
                State &t = state[cnt + 1];
                memcpy(&t, &s, sizeof(s));
                swap(t[z], t[z_new]);
                if (insert(cnt + 1)) {
                    dis[++cnt] = dis[index] + 1;
                    price[cnt] = dis[cnt] + cal_price(t);
                    parent[cnt] = index;
                    fout << index << /* "," << price[index] << */ " -> " << cnt << /*"," << price[cnt] << */ ";" << endl;
                    states.push(cnt);
                }
            }
        }
    }
    fout << "}" << endl;
    fout.close();
    return 0;
}

void print_board(State &s) {
    for (int i = 0; i < 9; ++i) {
        if (i % 3 == 0 && i > 0) {
            cout << endl;
        }
        cout << s[i] << " ";
    }
    cout << endl;
}

void print_paths(int start, int end) {
    int count = 1;
    stack<int> stack;
    while (start != end) {
        stack.push(end);
        end = parent[end];
    }
    while (!stack.empty()) {
        int i = stack.top();
        stack.pop();
        cout << "Step " << count++ << " : " << endl;
        print_board(state[i]);
    }
}

int check_rev(const State &now) {
    int rev = 0;
    for (int i = 0; i < 9; ++i) {
        for (int j = i + 1; j < 9; ++j) {
            if (now[i] != 0 && now[j] != 0 && now[i] > now[j]) {
                ++rev;
            }
        }
    }
    return rev;
}

// 1 2 3 4 5 0 7 8 6
// 2 3 4 1 5 0 7 6 8

// the number of h(n)
const int N = 5;
const char* FUNC_NAME[] = { "misplace square", "sum eucledian distance square", "sum manhattan distance square", "sum row col square", "nilsson sequence score" };

int main() {
    cout << "Please input start state: ";
    for (int i = 0; i < 9; ++i) {
        cin >> state[1][i];
    }
    cout << "Please input goal state: ";
    for (int i = 0; i < 9; ++i) {
        cin >> goal[i];
    }
    cout << "\nStart: " << endl;
    print_board(state[1]);
    cout << "\nGoal:" << endl;
    print_board(goal);
    cout << endl;

    int (*cal_price)(const State &now) = choose_cal_price_fun(1);

    //check if ans exist
    if (check_rev(state[1]) % 2 != check_rev(goal) % 2) {
        cout << "No solution" << endl;
        return 0;
    }

    // start timing
    LARGE_INTEGER t1, t2, tc;
    QueryPerformanceFrequency(&tc);
    QueryPerformanceCounter(&t1);

    int ans = A_star(cal_price);

    // end timing
    QueryPerformanceCounter(&t2);
    double my_time = (double) (t2.QuadPart - t1.QuadPart) / (double) tc.QuadPart;

    if (ans > 0) {
        cout << "Total steps: " << dis[ans] << endl << endl;
        print_paths(1, ans);
        cout << "A*search time " << my_time << endl;
        cout << "function h(n) of " << FUNC_NAME[0] << "'s time is " << my_time << endl;
        for (int i = 2; i <= N; ++i) {
            QueryPerformanceCounter(&t1);
            cal_price = choose_cal_price_fun(i);
            ans = A_star(cal_price);
            QueryPerformanceCounter(&t2);
            my_time = (double)(t2.QuadPart - t1.QuadPart) / (double)tc.QuadPart;
            if (ans > 0) {
                cout << "function h(n) of " << FUNC_NAME[i - 1] << "'s time is " << my_time << endl;
            }
        }
        cout << "Drawing search tree..." << endl;
        system("dot -Tpng out.txt -o out.png");
        cout << "Success." << endl;
    } else {
        cout << "No Solution!" << endl;
    }
    
    return 0;
}

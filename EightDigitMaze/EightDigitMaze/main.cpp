#include <iostream>
#include <cstring>
#include <cmath>
#include <queue>
#include <stack>
#include <iomanip>
#include <fstream>

using namespace std;

typedef int State[9];

const int STATE = 362800 + 5;

State state[STATE], goal;
int dis[STATE], parent[STATE], price[STATE], head[STATE], next_edge[STATE];

auto comp_price = [](const int& a, const int& b) { return price[a] > price[b]; };
priority_queue<int, vector<int>, decltype(comp_price)> states(comp_price);

const int FAC[] = { 1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880 };

int cantor_hash(const State& s) {
	int res = 0;
	for (int i = 0; i < 9; ++i) {
		int smaller = 0;
		for (int j = i + 1; j < 9; ++j)
			smaller += (s[j] < s[i]);
		res += FAC[8 - i] * smaller;
	}
	return res;
}

bool insert(const int &now) {
	int hash = cantor_hash(state[now]), u = head[hash];
	while (u) {
		if (!memcmp(state[u], state[now], sizeof(State)))
			return false;
		u = next_edge[u];
	}
	next_edge[now] = head[hash], head[hash] = now;
	return true;
}

int cal_price(const State& now) {
	int price = 0;
	for (int i = 0; i < 9; ++i) {
		for (int j = 0; j < 9; ++j) {
			if (now[i] == goal[j] && i != j) {
				int x = (now[i] / 3 - goal[j] / 3), y = (now[i] % 3 - goal[j] % 3);
				price += x * x + y * y;
			}
		}
	}
	return price;
}

const int DX[] = { -1, 0, 1, 0 };
const int DY[] = { 0, -1, 0, 1 };

ofstream fout("out.txt");

int A_star() {
	int cnt = 1;
	states.push(cnt);
	while (!states.empty()) {
		int index = states.top();
		states.pop();
		State& s = state[index];
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
				State& t = state[cnt + 1];
				memcpy(&t, &s, sizeof(s));
				swap(t[z], t[z_new]);
				if (insert(cnt + 1)) {
					dis[++cnt] = dis[index] + 1;
					price[cnt] = dis[cnt] + cal_price(t);
					parent[cnt] = index;
					fout << index <</* "," << price[index] << */" -> " << cnt << /*"," << price[cnt] << */";" << endl;
					states.push(cnt);
				}
			}
		}
	}
	return 0;
}

void print_board(State& s) {
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
	//ofstream fout("out.txt");
	//fout << "digraph EightDigit{" << endl;

	while (start != end) {
		stack.push(end);
		end = parent[end];
	}
	while (!stack.empty()) {
		int i = stack.top();
		stack.pop();
		cout << "Step " << count++ << " : " << endl;
		//fout << "f(n) = " << price[i] << ";" << endl;
		print_board(state[i]);
	}
	//fout << "}" << endl;
	//system("dot -Tpng out.txt -o out.png");
}


//1 2 3 4 5 0 7 8 6
//2 3 4 1 5 0 7 6 8

int main() {
	for (int i = 0; i < 9; ++i) { 
		cin >> state[1][i];
	}
	for (int i = 0; i < 9; ++i) {
		cin >> goal[i];
	}
	cout << "\nStart: " << endl;
	print_board(state[1]);
	cout << "\nGoal:" << endl;
	print_board(goal);
	cout << endl;

	fout << "digraph EightDigit{" << endl;
	int ans = A_star();
	if (ans > 0) {
		printf("Total paths: %d\n\n", dis[ans]);
		print_paths(1, ans);
	}
	else {
		cout << "No Solution!" << endl;
	}
	fout << "}" << endl;
	fout.close();

	return 0;
}
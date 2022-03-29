#include <iostream>
#include <cstring>
#include <cmath>
#include <queue>
#include <stack>

#define MAXSTATE 10000000
#define HASHSIZE 1000003

using namespace std;

typedef int State[9];

static State st[MAXSTATE], goal;
static int dist[MAXSTATE], p[MAXSTATE], price[MAXSTATE];

struct cmp_price
{
	bool operator ()(int x, int y)
	{
		return price[x] > price[y];  //小的优先级高
	}
};

static priority_queue<int, vector<int>, cmp_price> states;

static const int dx[] = { -1, 0, 1, 0 };
static const int dy[] = { 0, -1, 0, 1 };

static int head[HASHSIZE], next_pos[MAXSTATE];

static void init_lookup_table() {
	memset(head, 0, sizeof(head));
}

static int hash_state(State& s) {
	int v = 0;

	for (int i = 0; i < 9; ++i) {
		v = v * 10 + s[i];
	}

	return v % HASHSIZE;
}

static int try_to_insert(int s) {
	int h = hash_state(st[s]);
	int u = head[h];
	while (u) {
		if (memcmp(st[u], st[s], sizeof(st[s])) == 0) {
			return 0;
		}
		u = next_pos[u];
	}
	next_pos[s] = head[h];
	head[h] = s;

	return 1;
}

static int cal_price(State& s) {
	int price = 0;

	for (int i = 0; i < 9; ++i) {
		for (int j = 0; j < 9; ++j) {
			if (s[i] == goal[j] && i != j) {
				int x = (s[i] / 3 - goal[j] / 3);
				int y = (s[i] % 3 - goal[j] % 3);
				price += (int)sqrt((double)(x * x + y * y));
			}
		}
	}

	return price;
}

static int dfs(int max_depth) {
	init_lookup_table();

	int cnt = 1;
	int depth = 1;

	states.push(cnt);
	while (!states.empty()) {
		int index = states.top();
		states.pop();
		State& s = st[index];

		if (memcmp(s, goal, sizeof(s)) == 0) {
			return index;
		}

		int z;
		for (z = 0; z < 9; ++z) {
			if (s[z] == 0) {
				break;
			}
		}

		++depth;
		int x = z / 3, y = z % 3;
		for (int d = 0; d < 4; ++d) {
			int newX = x + dx[d];
			int newY = y + dy[d];
			if (newX >= 0 && newX < 3 && newY >= 0 && newY < 3) {
				int newZ = newX * 3 + newY;

				State& t = st[cnt + 1];
				memcpy(&t, &s, sizeof(s));
				t[newZ] = s[z];
				t[z] = s[newZ];

				if (try_to_insert(cnt + 1) == 1) {
					++cnt;
					dist[cnt] = dist[index] + 1;
					price[cnt] = dist[cnt] + cal_price(t);
					p[cnt] = index;
					states.push(cnt);
				}
			}
		}
	}

	return 0;
}

static void print_board(State& s) {
	for (int i = 0; i < 9; ++i) {
		if (i % 3 == 0 && i > 0) {
			cout << endl;
		}
		cout << s[i] << " ";
	}
	cout << endl;
}

static void print_paths(int start, int end) {
	int count = 1;
	stack<int> stack;

	while (start != end) {
		stack.push(end);
		end = p[end];
	}
	while (!stack.empty()) {
		int i = stack.top();
		stack.pop();
		cout << "Step " << count++ << " : " << endl;
		print_board(st[i]);
	}
}

int main() {
	for (int i = 0; i < 9; ++i) { 
		cin >> st[1][i];
	}
	for (int i = 0; i < 9; ++i) {
		cin >> goal[i];
	}
	cout << "\nStart: " << endl;
	print_board(st[1]);
	cout << "\nGoal:" << endl;
	print_board(goal);
	cout << endl;

	int ans = dfs(4);
	if (ans > 0) {
		printf("Total paths: %d\n\n", dist[ans]);
		print_paths(1, ans);
	}
	else {
		cout << "No Solution!" << endl;
	}

	return 0;
}
#include<iostream>
#include<string>
#include<algorithm>
#include<vector>

using namespace std;
string printOne(string s1, string s2, int **dp) {
    string ans = "";
    int t1 = s1.length();
    int t2 = s2.length();
    while(t1 > 0 && t2 > 0) {
        if(s1[t1-1] == s2[t2-1]) {
            ans = s1[t1-1] + ans;
            t1--;
            t2--;
        }else if(dp[t1-1][t2] < dp[t1][t2-1])
        {
            t2--;
        }else {
            t1--;
        }
    }
    return ans;
}

void printMulti(string s1, string s2, int **dp, int t1, int t2, vector<string>& vt_str, string str) {
    if(t1 <= 0 || t2 <= 0) {
        if(find(vt_str.begin(), vt_str.end(), str) == vt_str.end()) vt_str.push_back(str);
        return;
    }
    if(s1[t1-1] == s2[t2-1]) {
        printMulti(s1, s2, dp, t1-1, t2-1, vt_str, s1[t1-1]+str);
    }else if(dp[t1-1][t2] == dp[t1][t2-1]) {
        printMulti(s1, s2, dp, t1-1, t2, vt_str, str);
        printMulti(s1, s2, dp, t1, t2-1, vt_str, str);
    }else if(dp[t1-1][t2] < dp[t1][t2-1]) {
        printMulti(s1, s2, dp, t1, t2-1, vt_str, str);
    }else {
        printMulti(s1, s2, dp, t1-1, t2, vt_str, str);
    }
}
int LCS(string s1, string s2)
{
    int len1 = s1.length();
    int len2 = s2.length();
    int **dp = new int*[len1+1];
    for(int i = 0; i <= len1; i++) {
        dp[i] = new int[len2+1];
    }
    for(int i = 0; i <= len1; i++) dp[i][0] = 0;
    for(int i = 0; i <= len2; i++) dp[0][i] = 0;
    for(int i = 1; i <= len1; i++) {
        for(int j = 1; j <= len2; j++) {
            if(s1[i-1] == s2[j-1]) {
                dp[i][j] = dp[i-1][j-1] + 1;
            }else {
                dp[i][j] = max(dp[i-1][j], dp[i][j-1]);
            }
        }
    }
    int ans = dp[len1][len2];
    string str = printOne(s1, s2, dp);
    vector<string> vt_str;
    printMulti(s1, s2, dp, len1, len2, vt_str, "");
    cout<<"最长公共子序列如下:"<<endl;
    for(int i = 0; i < vt_str.size(); i++) cout<<vt_str[i]<<endl;
    for(int i = 0; i <= len1; i++) delete[] dp[i];
    delete[] dp;
    return ans;
}
int main()
{
    string s1, s2;
    s1="BDCABA";
    s2="ABCBDAB";
    /*while(cin>>s1>>s2) {
        cout<<LCS(s1, s2)<<endl;
    }*/
    cout<<"字符串s1为："<<s1<<endl;
    cout<<"字符串s2为："<<s2<<endl;
    int a = LCS(s1, s2);
    cout<<"最长公共子序列长度为："<<a<<endl;
}

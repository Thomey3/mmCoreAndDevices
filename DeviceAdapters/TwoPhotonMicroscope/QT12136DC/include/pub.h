#pragma once
#include <string>

using namespace std;

class pub
{
public:
	static std::string dec2hex(int i, int width);
	static string DecIntToHexStr(long long num);
	static string DecStrToHexStr(string str);
	static int convertBinaryToDecimal(long long n);
	static long long convertDecimalToBinary(int n);
	static string charToBin(char temp);
	static int stringToDouble(string temp);
	static double BenToDex(string temp);
	static double putin(string temp1);
};


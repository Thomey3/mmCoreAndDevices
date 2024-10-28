#include "../include/pub.h"
#include <sstream>

//i要转化的十进制整数，width转化后的宽度，位数不足则补0
std::string pub::dec2hex(int i, int width)
{
	std::stringstream ioss;     //定义字符串流
	std::string s_temp;         //存放转化后字符
	ioss << std::hex << i;      //以十六制形式输出
	ioss >> s_temp;

	if (width > s_temp.size())
	{
		std::string s_0(width - s_temp.size(), '0');      //位数不够则补0
		s_temp = s_0 + s_temp;                            //合并
	}

	std::string s = s_temp.substr(s_temp.length() - width, s_temp.length());    //取右width位
	return s;
}

string pub::DecIntToHexStr(long long num)			//十进制整数到十六进制字符的转换
{
	string str;
	long long Temp = num / 16;
	int left = num % 16;
	if (Temp > 0)
		str += DecIntToHexStr(Temp);
	if (left < 10)
		str += (left + '0');
	else
		str += ('A' + left - 10);
	return str;
}


string pub::DecStrToHexStr(string str)			//十进制字符到十六进制字符转换
{
	long long Dec = 0;
	for (int i = 0; i < str.size(); ++i)
		Dec = Dec * 10 + str[i] - '0';
	return DecIntToHexStr(Dec);
}


int pub::convertBinaryToDecimal(long long n)        //二进制转换为十进制
{
	int decimalNumber = 0, i = 0, remainder;
	while (n != 0)
	{
		remainder = n % 10;
		n /= 10;
		decimalNumber += remainder * pow(2, i);
		++i;
	}
	return decimalNumber;
}

long long pub::convertDecimalToBinary(int n)            //十进制转换为二进制
{
	long long binaryNumber = 0;
	int remainder, i = 1, step = 1;

	while (n != 0)
	{
		remainder = n % 2;
		printf("Step %d: %d/2, 余数 = %d, 商 = %d\n", step++, n, remainder, n / 2);
		n /= 2;
		binaryNumber += remainder * i;
		i *= 10;
	}
	return binaryNumber;
}

string pub::charToBin(char temp)//十六进制转二进制串
{
	switch (temp)
	{
	case '0':
		return "0000";
		break;
	case '1':
		return "0001";
		break;
	case '2':
		return "0010";
		break;
	case '3':
		return "0011";
		break;
	case '4':
		return "0100";
		break;
	case '5':
		return "0101";
		break;
	case '6':
		return "0110";
		break;
	case '7':
		return "0111";
		break;
	case '8':
		return "1000";
		break;
	case '9':
		return "1001";
		break;
	case 'A':case 'a':
		return "1010";
		break;
	case 'B':case 'b':
		return "1011";
		break;
	case 'C':case 'c':
		return "1100";
		break;
	case 'D':case 'd':
		return "1101";
		break;
	case 'E':case 'e':
		return "1110";
		break;
	case 'F':case 'f':
		return "1111";
		break;
	default:
		return "WRONG!";
	}
}

int pub::stringToDouble(string temp)//二进制串到double（整数）
{
	double res = 0;
	for (int i = 0; i < temp.length(); i++)
	{
		res = res * 2 + (temp[i] - '0');
	}
	return res;
}

double pub::BenToDex(string temp)//二进制串到double（小数）
{
	int m = temp.length();
	double res = 0;
	for (int i = 0; i < m; i++)
	{
		res = res + (temp[i] - '0') * pow(2, -i - 1);
	}
	return res;
}

double pub::putin(string temp1)
{
	int i;
	string temp;
	temp = temp1;
	string S_Bin = "";//转化后的二进制字符串
//cout<<temp.length()<<endl;//字符串长度16位
	for (int i = 0; i < temp.length(); i++)
	{
		char temp1 = temp[i];
		S_Bin = S_Bin + charToBin(temp1);//十六进制转二进制串
	}

	//  cout<<"转化后的二进制串 "<<S_Bin<<endl;//
	int sign = 0;//符号位
	if (S_Bin.at(0) == '1')//判断符号位
	{
		sign = 1;
	}
	string exponent = ""; //定义指数部分Exp
	for (i = 1; i < 12; i++)
	{
		if (S_Bin.at(i) == '1')
		{
			exponent = exponent + '1';
		}
		else
			exponent = exponent + '0';
	}
	int exponent_double = 0;//阶码
	exponent_double = stringToDouble(exponent);
	// cout<<"偏移位数 "<<exponent_double<<endl;
	exponent_double = exponent_double - 1023;//计算出偏移位数 ，64位双精度浮点数的偏置码为1023
   // cout<<"偏移位数 "<<exponent_double<<endl;
	string mantissa_temp = ""; //定义尾数部分Fraction
	for (i = 12; i < 64; i++)
	{
		if (S_Bin.at(i) == '1')
		{
			mantissa_temp = mantissa_temp + '1';
		}
		else
			mantissa_temp = mantissa_temp + '0';
	}
	double mantissa = 0;
	mantissa = BenToDex(mantissa_temp); //二进制串到double（小数）
  //  cout<<"小数点 "<<mantissa<<endl;
	mantissa = mantissa + 1.0;
	double res = 0;
	double a, c;
	a = pow((-1), sign);
	c = pow(2, exponent_double);
	res = a * mantissa * c;
	return res;
}

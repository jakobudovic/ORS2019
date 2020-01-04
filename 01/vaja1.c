#include <stdio.h>

int resetBit(int x, int p)
{

	x = x & ~(1 << p);
	return x;
}

int resetTwoBits(int x, int p)
{

	if (p > 1)
	{
		x = x & ~(1 << p);
		x = x & ~(1 << (p + 1));
	}
	else
	{
		x = x & ~(1 << p);
	}

	return x;
}

int setBit(int x, int p)
{
	// p-ti bit nastavimo na 1, ostali so na 0
	// uporabimo "or"

	x = x | (1 << p);
	return x;
}

int setTwoBitsTo(int x, int p, int n)
{
	x = resetTwoBits(x, p);
	if (n == 0)
	{
		// p-ti bit na 00
		return x;
	}
	else if (n == 1)
	{
		// p-ti bit na 01
		x = setBit(x, p - 1);
		return x;
	}
	else if (n == 2)
	{
		// p-ti bit na 10
		x = setBit(x, p);
		return x;
	}
	else if (n == 3)
	{
		// p-ti bit na 11
		x = setBit(x, p);
		x = setBit(x, p - 1);
		return x;
	}
	else
	{
		printf("error se je pojavil");
		return 0;
	}
}

int main()
{

	int x;
	scanf("%d", &x);
	int bit;
	scanf("%d", &bit);
	int n;
	scanf("%d", &n);

	int rez1 = resetBit(x, bit);
	int rez2 = resetTwoBits(x, bit);
	int rez3 = setBit(x, bit);
	int rez4 = setTwoBitsTo(x, bit, n);
	printf("%d\n", rez4);

	return 0;
}
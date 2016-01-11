
#include <windows.h>
#include <stdio.h>
#include <tchar.h>

/////////////////////////////////////////////////////////////////////////////////////////////

#include <geometry.h>

#define OUTLOOPCOUNT 16
#define LOOPCOUNT 2048*2048

static LARGE_INTEGER frequency;
static LARGE_INTEGER start;
static LARGE_INTEGER end;
static double interval;
static V3 values[LOOPCOUNT];
static V3* allocvalues;
static float dotproducts[LOOPCOUNT];

DECLSPEC_NOINLINE static const void UpdateInterval()
{
	interval = (double)(end.QuadPart - start.QuadPart) / (double)frequency.QuadPart;
}

/////////////////////////////////////////////////////////////////////////////////////////////

DECLSPEC_NOINLINE static const void GenerateRandoms()
{	
	// seed randomgen with current timestamp
	QueryPerformanceCounter(&start);
	srand(start.LowPart);

	for (size_t i = 0; i < LOOPCOUNT; i++)
	{
		values[i].X = allocvalues[i].X = (float)rand() * (1.0f / (float)RAND_MAX) * 10.0f;
		values[i].Y = allocvalues[i].Y = (float)rand() * (1.0f / (float)RAND_MAX) * 10.0f;
		values[i].Z = allocvalues[i].Z = (float)rand() * (1.0f / (float)RAND_MAX) * 10.0f;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////
DECLSPEC_NOINLINE static const void TestLoopEmpty()
{
	QueryPerformanceCounter(&start);
	__asm
	{
		mov ecx, OUTLOOPCOUNT

		outerloop:
			mov eax, LOOPCOUNT - 7

		innerloop:
			dec     eax
			jnz     innerloop

		dec ecx
		jnz outerloop
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("EMPTY LOOP   Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestLoopNop()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { nop }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("NOP          Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVAPSFrom()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVAPS xmm0, xmmword ptr values[0] }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVAPS FROM  Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVAPSTo()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVAPS xmmword ptr values[0], xmm0 }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVAPS TO    Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVUPSFrom()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVUPS xmm0, values[0] }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVUPS FROM  Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVUPSTo()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVUPS values[0], xmm0 }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVUPS TO    Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVSSFrom()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVSS xmm0, dword ptr values[0] }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVSS FROM   Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVSSTo()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		__asm { MOVSS dword ptr values[0], xmm0 }
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVSS TO     Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestADDPS()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
		for (size_t i = 7; i < LOOPCOUNT; i++)
		{
			__asm { ADDPS xmm0, xmm0 }
		}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("ADDPS        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestADDSS()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
		for (size_t i = 7; i < LOOPCOUNT; i++)
		{
			__asm { ADDSS xmm0, xmm0 }
		}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("ADDSS        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMOVADDMOV()
{
	__m128 val[3];

	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	{
#if 0
		__asm
		{
			mov eax, LOOPCOUNT - 7

			innerloop:
				movaps  xmm0, xmmword ptr values[TYPE val * 0]
				addps   xmm0, xmmword ptr values[TYPE val * 1]
				movaps  xmmword ptr values[TYPE val * 2], xmm0
				dec     eax
				jnz     innerloop
		}
#else
		for (size_t i = 7; i < LOOPCOUNT; i++)
		{
			__asm
			{
				movaps  xmm0, xmmword ptr values[TYPE val * 0]
				addps   xmm0, xmmword ptr values[TYPE val * 1]
				movaps  xmmword ptr values[TYPE val * 2], xmm0
			}
		}
#endif
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MOVADDMOV    Time: %f\n", interval);
}
/////////////////////////////////////////////////////////////////////////////////////////////

DECLSPEC_NOINLINE static const void TestV3Add()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V3ADD(&values[i - 1], &values[i - 5], &values[i - 3]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3ADD        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Sub()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V3SUB(&values[i - 1], &values[i - 5], &values[i - 3]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3SUB        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Dot()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		dotproducts[i] = V3DOT(&values[i - 5], &values[i - 3]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3DOT        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Cross()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V3CROSS(&values[i - 1], &values[i - 5], &values[i - 3]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3CROSS      Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3IsZero()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		*(int*)&dotproducts = V3ISZERO(&values[i - 6]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3ISZERO     Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Scale()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V3SCALE(&values[i-1], values[i].X);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3SCALE      Time: %f \n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Len()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		dotproducts[i] = V3LEN(&values[i-1]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3LEN        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV3Normalize()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V3NORMALIZE(&values[i - 1]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V3NORMALIZE  Time: %f\n", interval);
}
/////////////////////////////////////////////////////////////////////////////////////////////

DECLSPEC_NOINLINE static const void TestV2Add()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V2ADD(&values[i - 1], &values[i - 3], &values[i - 5]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V2ADD        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV2Sub()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V2SUB(&values[i - 1], &values[i - 3], &values[i - 5]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V2SUB        Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV2Scale()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		V2SCALE(&values[i - 1], values[i].X);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V2SCALE      Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestV2IsInBox()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		*(int*)&dotproducts[i] = ISINBOX(&values[i - 1], &values[i-3], &values[i-5]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("V2ISINBOX    Time: %f\n", interval);
}
/////////////////////////////////////////////////////////////////////////////////////////////

DECLSPEC_NOINLINE static const void TestIntersectLineTriangle()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		dotproducts[i] = IntersectLineTriangle(&allocvalues[i - 7], &allocvalues[i - 6], &allocvalues[i - 5], &values[i - 4], &values[i - 3]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("INT-LIN-TRIA Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestMinSquaredDistanceToLineSegment()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		dotproducts[i] = MinSquaredDistanceToLineSegment((V2*)&values[i - 7], (V2*)&allocvalues[i - 6], (V2*)&allocvalues[i - 5]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("MIN-DIST-LIN Time: %f\n", interval);
}

DECLSPEC_NOINLINE static const void TestIntersectLineCircle()
{
	QueryPerformanceCounter(&start);
	for (size_t j = 0; j < OUTLOOPCOUNT; j++)
	for (size_t i = 7; i < LOOPCOUNT; i++)
	{
		dotproducts[i] = IntersectLineCircle((V2*)&values[i - 7], 2.0f, (V2*)&allocvalues[i - 6], (V2*)&allocvalues[i - 5]);
	}
	QueryPerformanceCounter(&end);
	UpdateInterval();
	printf("INT-LIN-CIRC Time: %f\n", interval);
}
/////////////////////////////////////////////////////////////////////////////////////////////

int _tmain(int argc, _TCHAR* argv[])
{
	char str[64];
	sprintf_s(str, "LOOPS: %i\n", OUTLOOPCOUNT*LOOPCOUNT);
	printf(str);

	allocvalues = (V3*)_aligned_malloc(LOOPCOUNT * sizeof(V3), 16);

	QueryPerformanceFrequency(&frequency);

	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
	
	while (1)
	{
		system("cls");
		////////////////////////////////////////////////////////////////////////////////////
		GenerateRandoms();
		TestLoopEmpty();
		GenerateRandoms();
		TestLoopNop();
		GenerateRandoms();
		TestMOVAPSFrom();
		GenerateRandoms();
		TestMOVAPSTo();
		GenerateRandoms();
		TestMOVUPSFrom();
		GenerateRandoms();
		TestMOVUPSTo();
		GenerateRandoms();
		TestMOVSSFrom();
		GenerateRandoms();
		TestMOVSSTo();
		GenerateRandoms();
		TestADDPS();
		GenerateRandoms();
		TestADDSS();
		GenerateRandoms();
		TestMOVADDMOV();
		////////////////////////////////////////////////////////////////////////////////////
		printf("---------------------------------------------\n");
		GenerateRandoms();
		TestV3IsZero();
		GenerateRandoms();
		TestV3Add();
		GenerateRandoms();
		TestV3Sub();
		GenerateRandoms();
		TestV3Scale();
		GenerateRandoms();
		TestV3Dot();
		GenerateRandoms();
		TestV3Cross();
		GenerateRandoms();
		TestV3Len();
		GenerateRandoms();
		TestV3Normalize();
		////////////////////////////////////////////////////////////////////////////////////
		printf("---------------------------------------------\n");
		GenerateRandoms();
		TestV2Add();
		GenerateRandoms();
		TestV2Sub();
		GenerateRandoms();
		TestV2Scale();
		GenerateRandoms();
		TestV2IsInBox();
		////////////////////////////////////////////////////////////////////////////////////
		printf("---------------------------------------------\n");
		GenerateRandoms();
		TestIntersectLineTriangle();
		GenerateRandoms();
		TestMinSquaredDistanceToLineSegment();
		GenerateRandoms();
		TestIntersectLineCircle();
		////////////////////////////////////////////////////////////////////////////////////

		printf("---------------------------------------------\n");
		printf("          PRESS KEY TO RESTART \n");
		getchar();
	}

	return 0;
}


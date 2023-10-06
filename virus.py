
#초기 바이러스 개수 = K , 초당 P배 증가, N초 후 => K*(p**N) = Total number of viruses

K, P, N = map(int, input("").split())

#if 1 <= K <= 10**8 and 1 <= P <= 10**8 and 1 <= N <= 10**8:
if K in range(1, 10**8+1) and P in range(1, 10**8+1) and N in range(1, 10**8+1):
    print(K*(P**N) % 1000000007)
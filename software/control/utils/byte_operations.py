from numpy import std, square, mean


def unsigned_to_signed(unsigned_array,N):
    signed = 0
    for i in range(N):
        signed = signed + int(unsigned_array[i])*(256**(N-1-i))
    signed = signed - (256**N)/2
    return signed

def split_int_2byte(number):
    return int(number)% 256,int(number) >> 8

def split_signed_int_2byte(number):
    if abs(number) > 32767:
        number = np.sign(number)*32767

    if number!=abs(number):
        number=65536+number
    return int(number)% 256,int(number) >> 8

def split_int_3byte(number):
    return int(number)%256, int(number) >> 8, int(number) >> 16

def data2byte_to_int(a,b):
    return a + 256*b

def data2byte_to_signed_int(a,b):
    nb= a+256*b
    if nb>32767:
        nb=nb-65536
    return nb

def data4byte_to_int(a,b,c,d):
    return a + (256)*b + (65536)*c + (16777216)*d
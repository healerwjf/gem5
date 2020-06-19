#include <stdio.h>
#include <stdlib.h>

int main() {
    int a[4][4], b[4][4], c[4][4];
    
    //数组a初始化
    for (int i = 0; i < 4; ++i) {
    	for (int j = 0; j < 4; ++j) {
    	    a[i][j] = 1;
    	}
    }
    
    //数组b初始化
    for (int i = 0; i < 4; ++i) {
    	for (int j = 0; j < 4; ++j) {
    	    b[i][j] = 2;
    	}
    }
    
    //数组c初始化
    for (int i = 0; i < 4; ++i) {
    	for (int j = 0; j < 4; ++j) {
    	    c[i][j] = 0;
    	}
    }
    
    //实现c = a × b
    for (int i = 0; i < 4; ++i) {
    	for (int j = 0; j < 4; ++j) {
    	    for (int k = 0; k < 4; ++k) {
    	    	c[i][j] += a[i][k] * b[k][j];
    	    }
    	}
    }
    
    //打印结果
    for (int i = 0; i < 4; ++i) {
    	for (int j = 0; j < 4; ++j) {
    	    printf("c[%d][%d] = %d\n", i, j, c[i][j]);
    	}
    }
    
    return 0;
}

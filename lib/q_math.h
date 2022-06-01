/* 
============================================================================================================================
q_math.c(q_math.hと拡張子を変更して使用して下さい)
Copyright (C) 2015 清宮 宏
ソースコード形式かバイナリ形式か、変更するかしないかを問わず、以下の条件を満たす場合に限り、再頒布および使用が許可されます。

* ソースコードを再頒布する場合、上記の著作権表示、本条件一覧、および下記免責条項を含めること。
* バイナリ形式で再頒布する場合、頒布物に付属のドキュメント等の資料に、上記の著作権表示、本条件一覧、および下記免責条項を含めること。
* 書面による特別の許可なしに、本ソフトウェアから派生した製品の宣伝または販売促進に、著作権者の名前を使用してはならない。

本ソフトウェアは無保証であり、自己責任で使用して下さい。
本ソフトウェアの使用によって生じた損害について、著作権者は一切責任を負わないものとします。
============================================================================================================================
*/

//関数マクロ定義


#define   Q_MUL(a,b,n)	((long)(((long long)(a) * (long long)(b)+(1<<((n)-1)))>>(n)))

#define   Q_MUL_SH(a,b,n)	(((a) * (b)+(1<<((n)-1)))>>(n))

#define   Q_MUL_FR(a,b,n)	((((n)>15) ? Q_MUL(a,b,n) :Q_MUL_SH(a,b,n)) & (0x7FFFFFFF >> (31-(n))))
	
#define   Q_PDIV(a,b,n)  ((long)((((long long)(a) << (n)) + ((b) >>1))/(b)))

#define Q_DIV(a,b,n)	 (\
 	(((a) >= 0 && (b) >= 0) || ((a) < 0 && (b) < 0)) ?\
 	((long)((((long long)(a) << (n)) + ((b) >>1))/(b))) : \
  	((long)((((long long)(a) << (n)) - ((b) >>1))/(b)))\
  )


#define		Q1_MUL(a,b)		Q_MUL(a,b,1)
#define		Q2_MUL(a,b)		Q_MUL(a,b,2)
#define		Q3_MUL(a,b)		Q_MUL(a,b,3)
#define		Q4_MUL(a,b)		Q_MUL(a,b,4)
#define		Q5_MUL(a,b)		Q_MUL(a,b,5)
#define		Q6_MUL(a,b)		Q_MUL(a,b,6)
#define		Q7_MUL(a,b)		Q_MUL(a,b,7)
#define		Q8_MUL(a,b)		Q_MUL(a,b,8)
#define		Q9_MUL(a,b)		Q_MUL(a,b,9)
#define		Q10_MUL(a,b)	Q_MUL(a,b,10)
#define		Q11_MUL(a,b)	Q_MUL(a,b,11)
#define		Q12_MUL(a,b)	Q_MUL(a,b,12)
#define		Q13_MUL(a,b)	Q_MUL(a,b,13)
#define		Q14_MUL(a,b)	Q_MUL(a,b,14)
#define		Q15_MUL(a,b)	Q_MUL(a,b,15)
#define		Q16_MUL(a,b)	Q_MUL(a,b,16)
#define		Q17_MUL(a,b)	Q_MUL(a,b,17)
#define		Q18_MUL(a,b)	Q_MUL(a,b,18)
#define		Q19_MUL(a,b)	Q_MUL(a,b,19)
#define		Q20_MUL(a,b)	Q_MUL(a,b,20)
#define		Q21_MUL(a,b)	Q_MUL(a,b,21)
#define		Q22_MUL(a,b)	Q_MUL(a,b,22)
#define		Q23_MUL(a,b)	Q_MUL(a,b,23)
#define		Q24_MUL(a,b)	Q_MUL(a,b,24)
#define		Q25_MUL(a,b)	Q_MUL(a,b,25)
#define		Q26_MUL(a,b)	Q_MUL(a,b,26)
#define		Q27_MUL(a,b)	Q_MUL(a,b,27)
#define		Q28_MUL(a,b)	Q_MUL(a,b,28)
#define		Q29_MUL(a,b)	Q_MUL(a,b,29)
#define		Q30_MUL(a,b)	Q_MUL(a,b,30)
#define		Q31_MUL(a,b)	Q_MUL(a,b,31)


#define		Q1_MUL_SH(a,b)	Q_MUL_SH(a,b,1)
#define		Q2_MUL_SH(a,b)	Q_MUL_SH(a,b,2)
#define		Q3_MUL_SH(a,b)	Q_MUL_SH(a,b,3)
#define		Q4_MUL_SH(a,b)	Q_MUL_SH(a,b,4)
#define		Q5_MUL_SH(a,b)	Q_MUL_SH(a,b,5)
#define		Q6_MUL_SH(a,b)	Q_MUL_SH(a,b,6)
#define		Q7_MUL_SH(a,b)	Q_MUL_SH(a,b,7)
#define		Q8_MUL_SH(a,b)	Q_MUL_SH(a,b,8)
#define		Q9_MUL_SH(a,b)	Q_MUL_SH(a,b,9)
#define		Q10_MUL_SH(a,b)	Q_MUL_SH(a,b,10)
#define		Q11_MUL_SH(a,b)	Q_MUL_SH(a,b,11)
#define		Q12_MUL_SH(a,b)	Q_MUL_SH(a,b,12)
#define		Q13_MUL_SH(a,b)	Q_MUL_SH(a,b,13)
#define		Q14_MUL_SH(a,b)	Q_MUL_SH(a,b,14)
#define		Q15_MUL_SH(a,b)	Q_MUL_SH(a,b,15)
#define		Q16_MUL_SH(a,b)	Q_MUL_SH(a,b,16)
#define		Q17_MUL_SH(a,b)	Q_MUL_SH(a,b,17)
#define		Q18_MUL_SH(a,b)	Q_MUL_SH(a,b,18)
#define		Q19_MUL_SH(a,b)	Q_MUL_SH(a,b,19)
#define		Q20_MUL_SH(a,b)	Q_MUL_SH(a,b,20)
#define		Q21_MUL_SH(a,b)	Q_MUL_SH(a,b,21)
#define		Q22_MUL_SH(a,b)	Q_MUL_SH(a,b,22)
#define		Q23_MUL_SH(a,b)	Q_MUL_SH(a,b,23)
#define		Q24_MUL_SH(a,b)	Q_MUL_SH(a,b,24)
#define		Q25_MUL_SH(a,b)	Q_MUL_SH(a,b,25)
#define		Q26_MUL_SH(a,b)	Q_MUL_SH(a,b,26)
#define		Q27_MUL_SH(a,b)	Q_MUL_SH(a,b,27)
#define		Q28_MUL_SH(a,b)	Q_MUL_SH(a,b,28)
#define		Q29_MUL_SH(a,b)	Q_MUL_SH(a,b,29)
#define		Q30_MUL_SH(a,b)	Q_MUL_SH(a,b,30)
#define		Q31_MUL_SH(a,b)	Q_MUL_SH(a,b,31)


#define		Q1_MUL_FR(a,b)	Q_MUL_FR(a,b,1)
#define		Q2_MUL_FR(a,b)	Q_MUL_FR(a,b,2)
#define		Q3_MUL_FR(a,b)	Q_MUL_FR(a,b,3)
#define		Q4_MUL_FR(a,b)	Q_MUL_FR(a,b,4)
#define		Q5_MUL_FR(a,b)	Q_MUL_FR(a,b,5)
#define		Q6_MUL_FR(a,b)	Q_MUL_FR(a,b,6)
#define		Q7_MUL_FR(a,b)	Q_MUL_FR(a,b,7)
#define		Q8_MUL_FR(a,b)	Q_MUL_FR(a,b,8)
#define		Q9_MUL_FR(a,b)	Q_MUL_FR(a,b,9)
#define		Q10_MUL_FR(a,b)	Q_MUL_FR(a,b,10)
#define		Q11_MUL_FR(a,b)	Q_MUL_FR(a,b,11)
#define		Q12_MUL_FR(a,b)	Q_MUL_FR(a,b,12)
#define		Q13_MUL_FR(a,b)	Q_MUL_FR(a,b,13)
#define		Q14_MUL_FR(a,b)	Q_MUL_FR(a,b,14)
#define		Q15_MUL_FR(a,b)	Q_MUL_FR(a,b,15)
#define		Q16_MUL_FR(a,b)	Q_MUL_FR(a,b,16)
#define		Q17_MUL_FR(a,b)	Q_MUL_FR(a,b,17)
#define		Q18_MUL_FR(a,b)	Q_MUL_FR(a,b,18)
#define		Q19_MUL_FR(a,b)	Q_MUL_FR(a,b,19)
#define		Q20_MUL_FR(a,b)	Q_MUL_FR(a,b,20)
#define		Q21_MUL_FR(a,b)	Q_MUL_FR(a,b,21)
#define		Q22_MUL_FR(a,b)	Q_MUL_FR(a,b,22)
#define		Q23_MUL_FR(a,b)	Q_MUL_FR(a,b,23)
#define		Q24_MUL_FR(a,b)	Q_MUL_FR(a,b,24)
#define		Q25_MUL_FR(a,b)	Q_MUL_FR(a,b,25)
#define		Q26_MUL_FR(a,b)	Q_MUL_FR(a,b,26)
#define		Q27_MUL_FR(a,b)	Q_MUL_FR(a,b,27)
#define		Q28_MUL_FR(a,b)	Q_MUL_FR(a,b,28)
#define		Q29_MUL_FR(a,b)	Q_MUL_FR(a,b,29)
#define		Q30_MUL_FR(a,b)	Q_MUL_FR(a,b,30)
#define		Q31_MUL_FR(a,b)	Q_MUL_FR(a,b,31)


#define		Q1_DIV(a,b)		Q_DIV(a,b,1)
#define		Q2_DIV(a,b)		Q_DIV(a,b,2)
#define		Q3_DIV(a,b)		Q_DIV(a,b,3)
#define		Q4_DIV(a,b)		Q_DIV(a,b,4)
#define		Q5_DIV(a,b)		Q_DIV(a,b,5)
#define		Q6_DIV(a,b)		Q_DIV(a,b,6)
#define		Q7_DIV(a,b)		Q_DIV(a,b,7)
#define		Q8_DIV(a,b)		Q_DIV(a,b,8)
#define		Q9_DIV(a,b)		Q_DIV(a,b,9)
#define		Q10_DIV(a,b)	Q_DIV(a,b,10)
#define		Q11_DIV(a,b)	Q_DIV(a,b,11)
#define		Q12_DIV(a,b)	Q_DIV(a,b,12)
#define		Q13_DIV(a,b)	Q_DIV(a,b,13)
#define		Q14_DIV(a,b)	Q_DIV(a,b,14)
#define		Q15_DIV(a,b)	Q_DIV(a,b,15)
#define		Q16_DIV(a,b)	Q_DIV(a,b,16)
#define		Q17_DIV(a,b)	Q_DIV(a,b,17)
#define		Q18_DIV(a,b)	Q_DIV(a,b,18)
#define		Q19_DIV(a,b)	Q_DIV(a,b,19)
#define		Q20_DIV(a,b)	Q_DIV(a,b,20)
#define		Q21_DIV(a,b)	Q_DIV(a,b,21)
#define		Q22_DIV(a,b)	Q_DIV(a,b,22)
#define		Q23_DIV(a,b)	Q_DIV(a,b,23)
#define		Q24_DIV(a,b)	Q_DIV(a,b,24)
#define		Q25_DIV(a,b)	Q_DIV(a,b,25)
#define		Q26_DIV(a,b)	Q_DIV(a,b,26)
#define		Q27_DIV(a,b)	Q_DIV(a,b,27)
#define		Q28_DIV(a,b)	Q_DIV(a,b,28)
#define		Q29_DIV(a,b)	Q_DIV(a,b,29)
#define		Q30_DIV(a,b)	Q_DIV(a,b,30)
#define		Q31_DIV(a,b)	Q_DIV(a,b,31)


#define		Q1_PDIV(a,b)	Q_PDIV(a,b,1)
#define		Q2_PDIV(a,b)	Q_PDIV(a,b,2)
#define		Q3_PDIV(a,b)	Q_PDIV(a,b,3)
#define		Q4_PDIV(a,b)	Q_PDIV(a,b,4)
#define		Q5_PDIV(a,b)	Q_PDIV(a,b,5)
#define		Q6_PDIV(a,b)	Q_PDIV(a,b,6)
#define		Q7_PDIV(a,b)	Q_PDIV(a,b,7)
#define		Q8_PDIV(a,b)	Q_PDIV(a,b,8)
#define		Q9_PDIV(a,b)	Q_PDIV(a,b,9)
#define		Q10_PDIV(a,b)	Q_PDIV(a,b,10)
#define		Q11_PDIV(a,b)	Q_PDIV(a,b,11)
#define		Q12_PDIV(a,b)	Q_PDIV(a,b,12)
#define		Q13_PDIV(a,b)	Q_PDIV(a,b,13)
#define		Q14_PDIV(a,b)	Q_PDIV(a,b,14)
#define		Q15_PDIV(a,b)	Q_PDIV(a,b,15)
#define		Q16_PDIV(a,b)	Q_PDIV(a,b,16)
#define		Q17_PDIV(a,b)	Q_PDIV(a,b,17)
#define		Q18_PDIV(a,b)	Q_PDIV(a,b,18)
#define		Q19_PDIV(a,b)	Q_PDIV(a,b,19)
#define		Q20_PDIV(a,b)	Q_PDIV(a,b,20)
#define		Q21_PDIV(a,b)	Q_PDIV(a,b,21)
#define		Q22_PDIV(a,b)	Q_PDIV(a,b,22)
#define		Q23_PDIV(a,b)	Q_PDIV(a,b,23)
#define		Q24_PDIV(a,b)	Q_PDIV(a,b,24)
#define		Q25_PDIV(a,b)	Q_PDIV(a,b,25)
#define		Q26_PDIV(a,b)	Q_PDIV(a,b,26)
#define		Q27_PDIV(a,b)	Q_PDIV(a,b,27)
#define		Q28_PDIV(a,b)	Q_PDIV(a,b,28)
#define		Q29_PDIV(a,b)	Q_PDIV(a,b,29)
#define		Q30_PDIV(a,b)	Q_PDIV(a,b,30)
#define		Q31_PDIV(a,b)	Q_PDIV(a,b,31)


#define	Q_FROMW(a,n)	(((a)>=0)? ((long)((a)*(1<<(n))+0.5)) : ((long)((a)*(1<<(n))-0.5)))

#define	Q_FROMF(a,n)	(((a)>=0)? ((long)((a)*(1<<(n))+0.5)) : ((long)((a)*(1<<(n))-0.5)))

#define	Q_TOW(a,n)	((double)(a)*1.0/(1<<(n)))

#define	Q_TOF(a,n)	((float)(a)*1.0/(1<<(n)))


#define		Q1_FROMW(a)		Q_FROMW(a,1)
#define		Q2_FROMW(a)		Q_FROMW(a,2)
#define		Q3_FROMW(a)		Q_FROMW(a,3)
#define		Q4_FROMW(a)		Q_FROMW(a,4)
#define		Q5_FROMW(a)		Q_FROMW(a,5)
#define		Q6_FROMW(a)		Q_FROMW(a,6)
#define		Q7_FROMW(a)		Q_FROMW(a,7)
#define		Q8_FROMW(a)		Q_FROMW(a,8)
#define		Q9_FROMW(a)		Q_FROMW(a,9)
#define		Q10_FROMW(a)	Q_FROMW(a,10)
#define		Q11_FROMW(a)	Q_FROMW(a,11)
#define		Q12_FROMW(a)	Q_FROMW(a,12)
#define		Q13_FROMW(a)	Q_FROMW(a,13)
#define		Q14_FROMW(a)	Q_FROMW(a,14)
#define		Q15_FROMW(a)	Q_FROMW(a,15)
#define		Q16_FROMW(a)	Q_FROMW(a,16)
#define		Q17_FROMW(a)	Q_FROMW(a,17)
#define		Q18_FROMW(a)	Q_FROMW(a,18)
#define		Q19_FROMW(a)	Q_FROMW(a,19)
#define		Q20_FROMW(a)	Q_FROMW(a,20)
#define		Q21_FROMW(a)	Q_FROMW(a,21)
#define		Q22_FROMW(a)	Q_FROMW(a,22)
#define		Q23_FROMW(a)	Q_FROMW(a,23)
#define		Q24_FROMW(a)	Q_FROMW(a,24)
#define		Q25_FROMW(a)	Q_FROMW(a,25)
#define		Q26_FROMW(a)	Q_FROMW(a,26)
#define		Q27_FROMW(a)	Q_FROMW(a,27)
#define		Q28_FROMW(a)	Q_FROMW(a,28)
#define		Q29_FROMW(a)	Q_FROMW(a,29)
#define		Q30_FROMW(a)	Q_FROMW(a,30)
#define		Q31_FROMW(a)	Q_FROMW(a,31)


#define		Q1_FROMF(a)		Q_FROMF(a,1)
#define		Q2_FROMF(a)		Q_FROMF(a,2)
#define		Q3_FROMF(a)		Q_FROMF(a,3)
#define		Q4_FROMF(a)		Q_FROMF(a,4)
#define		Q5_FROMF(a)		Q_FROMF(a,5)
#define		Q6_FROMF(a)		Q_FROMF(a,6)
#define		Q7_FROMF(a)		Q_FROMF(a,7)
#define		Q8_FROMF(a)		Q_FROMF(a,8)
#define		Q9_FROMF(a)		Q_FROMF(a,9)
#define		Q10_FROMF(a)	Q_FROMF(a,10)
#define		Q11_FROMF(a)	Q_FROMF(a,11)
#define		Q12_FROMF(a)	Q_FROMF(a,12)
#define		Q13_FROMF(a)	Q_FROMF(a,13)
#define		Q14_FROMF(a)	Q_FROMF(a,14)
#define		Q15_FROMF(a)	Q_FROMF(a,15)
#define		Q16_FROMF(a)	Q_FROMF(a,16)
#define		Q17_FROMF(a)	Q_FROMF(a,17)
#define		Q18_FROMF(a)	Q_FROMF(a,18)
#define		Q19_FROMF(a)	Q_FROMF(a,19)
#define		Q20_FROMF(a)	Q_FROMF(a,20)
#define		Q21_FROMF(a)	Q_FROMF(a,21)
#define		Q22_FROMF(a)	Q_FROMF(a,22)
#define		Q23_FROMF(a)	Q_FROMF(a,23)
#define		Q24_FROMF(a)	Q_FROMF(a,24)
#define		Q25_FROMF(a)	Q_FROMF(a,25)
#define		Q26_FROMF(a)	Q_FROMF(a,26)
#define		Q27_FROMF(a)	Q_FROMF(a,27)
#define		Q28_FROMF(a)	Q_FROMF(a,28)
#define		Q29_FROMF(a)	Q_FROMF(a,29)
#define		Q30_FROMF(a)	Q_FROMF(a,30)
#define		Q31_FROMF(a)	Q_FROMF(a,31)


#define		Q1_TOW(a)		Q_TOW(a,1)
#define		Q2_TOW(a)		Q_TOW(a,2)
#define		Q3_TOW(a)		Q_TOW(a,3)
#define		Q4_TOW(a)		Q_TOW(a,4)
#define		Q5_TOW(a)		Q_TOW(a,5)
#define		Q6_TOW(a)		Q_TOW(a,6)
#define		Q7_TOW(a)		Q_TOW(a,7)
#define		Q8_TOW(a)		Q_TOW(a,8)
#define		Q9_TOW(a)		Q_TOW(a,9)
#define		Q10_TOW(a)		Q_TOW(a,10)
#define		Q11_TOW(a)		Q_TOW(a,11)
#define		Q12_TOW(a)		Q_TOW(a,12)
#define		Q13_TOW(a)		Q_TOW(a,13)
#define		Q14_TOW(a)		Q_TOW(a,14)
#define		Q15_TOW(a)		Q_TOW(a,15)
#define		Q16_TOW(a)		Q_TOW(a,16)
#define		Q17_TOW(a)		Q_TOW(a,17)
#define		Q18_TOW(a)		Q_TOW(a,18)
#define		Q19_TOW(a)		Q_TOW(a,19)
#define		Q20_TOW(a)		Q_TOW(a,20)
#define		Q21_TOW(a)		Q_TOW(a,21)
#define		Q22_TOW(a)		Q_TOW(a,22)
#define		Q23_TOW(a)		Q_TOW(a,23)
#define		Q24_TOW(a)		Q_TOW(a,24)
#define		Q25_TOW(a)		Q_TOW(a,25)
#define		Q26_TOW(a)		Q_TOW(a,26)
#define		Q27_TOW(a)		Q_TOW(a,27)
#define		Q28_TOW(a)		Q_TOW(a,28)
#define		Q29_TOW(a)		Q_TOW(a,29)
#define		Q30_TOW(a)		Q_TOW(a,30)
#define		Q31_TOW(a)		Q_TOW(a,31)


#define		Q1_TOF(a)		Q_TOF(a,1)
#define		Q2_TOF(a)		Q_TOF(a,2)
#define		Q3_TOF(a)		Q_TOF(a,3)
#define		Q4_TOF(a)		Q_TOF(a,4)
#define		Q5_TOF(a)		Q_TOF(a,5)
#define		Q6_TOF(a)		Q_TOF(a,6)
#define		Q7_TOF(a)		Q_TOF(a,7)
#define		Q8_TOF(a)		Q_TOF(a,8)
#define		Q9_TOF(a)		Q_TOF(a,9)
#define		Q10_TOF(a)		Q_TOF(a,10)
#define		Q11_TOF(a)		Q_TOF(a,11)
#define		Q12_TOF(a)		Q_TOF(a,12)
#define		Q13_TOF(a)		Q_TOF(a,13)
#define		Q14_TOF(a)		Q_TOF(a,14)
#define		Q15_TOF(a)		Q_TOF(a,15)
#define		Q16_TOF(a)		Q_TOF(a,16)
#define		Q17_TOF(a)		Q_TOF(a,17)
#define		Q18_TOF(a)		Q_TOF(a,18)
#define		Q19_TOF(a)		Q_TOF(a,19)
#define		Q20_TOF(a)		Q_TOF(a,20)
#define		Q21_TOF(a)		Q_TOF(a,21)
#define		Q22_TOF(a)		Q_TOF(a,22)
#define		Q23_TOF(a)		Q_TOF(a,23)
#define		Q24_TOF(a)		Q_TOF(a,24)
#define		Q25_TOF(a)		Q_TOF(a,25)
#define		Q26_TOF(a)		Q_TOF(a,26)
#define		Q27_TOF(a)		Q_TOF(a,27)
#define		Q28_TOF(a)		Q_TOF(a,28)
#define		Q29_TOF(a)		Q_TOF(a,29)
#define		Q30_TOF(a)		Q_TOF(a,30)
#define		Q31_TOF(a)		Q_TOF(a,31)


#define	Q_1 		( 1 << 1 ) 		// 2^ 1 
#define	Q_2 		( 1 << 2 ) 		// 2^ 2 
#define	Q_3 		( 1 << 3 ) 		// 2^ 3 
#define	Q_4 		( 1 << 4 ) 		// 2^ 4 
#define	Q_5 		( 1 << 5 ) 		// 2^ 5 
#define	Q_6 		( 1 << 6 ) 		// 2^ 6 
#define	Q_7 		( 1 << 7 ) 		// 2^ 7 
#define	Q_8 		( 1 << 8 ) 		// 2^ 8 
#define	Q_9 		( 1 << 9 ) 		// 2^ 9 
#define	Q_10 		( 1 << 10 ) 	// 2^ 10 
#define	Q_11 		( 1 << 11 ) 	// 2^ 11 
#define	Q_12 		( 1 << 12 ) 	// 2^ 12 
#define	Q_13 		( 1 << 13 ) 	// 2^ 13 
#define	Q_14 		( 1 << 14 ) 	// 2^ 14 
#define	Q_15 		( 1 << 15 ) 	// 2^ 15 
#define	Q_16 		( 1 << 16 ) 	// 2^ 16 
#define	Q_17 		( 1 << 17 ) 	// 2^ 17 
#define	Q_18 		( 1 << 18 ) 	// 2^ 18 
#define	Q_19 		( 1 << 19 ) 	// 2^ 19 
#define	Q_20 		( 1 << 20 ) 	// 2^ 20 
#define	Q_21 		( 1 << 21 ) 	// 2^ 21 
#define	Q_22 		( 1 << 22 ) 	// 2^ 22 
#define	Q_23 		( 1 << 23 ) 	// 2^ 23 
#define	Q_24 		( 1 << 24 ) 	// 2^ 24 
#define	Q_25 		( 1 << 25 ) 	// 2^ 25 
#define	Q_26 		( 1 << 26 ) 	// 2^ 26 
#define	Q_27 		( 1 << 27 ) 	// 2^ 27 
#define	Q_28 		( 1 << 28 ) 	// 2^ 28 
#define	Q_29 		( 1 << 29 ) 	// 2^ 29 
#define	Q_30 		( 1 << 30 ) 	// 2^ 30 
#define	Q_31 		0x80000000UL 	// 2^ 31 

#define Q_MAX		0x7FFFFFFF		//最大値

#define Q26_FLOOR		(0xFFFFFFFF<<26)//小数部分を切り捨てる
#define Q26_E			182420806		// 2.71828182845905	自然対数の底
#define Q26_LOG2E		96817625		// 1.44269504088896	1/log(2)
#define Q26_LOG10E		29145009		// .434294481903252	1/log(10)
#define Q26_LN2			46516320		// .693147180559945 log(2)
#define Q26_LN10		154523870		// 2.30258509299405 log(10)
#define Q26_PI    		210828714       // 3.14159265358979	円周率
#define Q26_PI_2  		105414357		// 1.5707963267949  円周率/2
#define Q26_PI_4		52707179		// .785398163397448	円周率/4
#define Q26_1_PI		21361415		// .318309886183791	1/円周率
#define Q26_2_PI		42722830		// .636619772367581	2/円周率
#define Q26_2_SQRTPI	75724244		// 1.12837916709551	2/sqrt(円周率)
#define Q26_SQRT2		94906266		// 1.4142135623731	sqrt(2)
#define Q26_SQRT1_2		47453133		// .707106781186548	1/sqrt(2)

#define Q23_FLOOR		(0xFFFFFFFF<<23)//小数部分を切り捨てる
#define Q23_E			22802601		// 2.71828182845905	自然対数の底
#define Q23_LOG2E		12102203		// 1.44269504088896	1/log(2)
#define Q23_LOG10E		3643126 		// .434294481903252	1/log(10)
#define Q23_LN2			5814540			// .693147180559945 log(2)
#define Q23_LN10		19315484 		// 2.30258509299405 log(10)
#define Q23_PI    		26353589        // 3.14159265358979	円周率
#define Q23_PI_2  		13176795 		// 1.5707963267949  円周率/2
#define Q23_PI_4		6588397			// .785398163397448	円周率/4
#define Q23_1_PI		2670177			// .318309886183791	1/円周率
#define Q23_2_PI		5340354			// .636619772367581	2/円周率
#define Q23_2_SQRTPI	9465531			// 1.12837916709551	2/sqrt(円周率)
#define Q23_SQRT2		11863283 		// 1.4142135623731	sqrt(2)
#define Q23_SQRT1_2		5931642			// .707106781186548	1/sqrt(2)

#define Q29_LN2			372130559		// .693147180559945 log(2)
#define Q29_PI    		1686629713      // 3.14159265358979	円周率 
#define Q29_PI_2  		843314857		// 1.5707963267949  円周率/2 


//q23関数定義 

long q23_mul(long,long);//注意　23bitと26bitの積は26bitになる 


long q23_div(long,long);

//床関数　floor(x) (-255<=x<=255) 
long q23_floor(long);

//天井関数　ceil(x) (-255<=x<=255) 
long q23_ceil(long);

//四捨五入　round(x) (-255<=x<=255) 
long q23_round(long);

//x>yのときx-yを返し，x≦yのとき0を返す．　fdim(x,y) (-255<=x<=255) 
long q23_dim(long,long);


//2つの引数xとyの最大値を返す．　fmax(x,y) (-255<=x<=255) 
long q23_max(long,long);

//2つの引数xとyの最小値を返す．　fmin(x,y) (-255<=x<=255) 
long q23_min(long,long);

//整数値に丸める関数　trunc(x) (-255<=x<=255) 
//小数点以下を0方向に切り捨てて整数値を返す
long q23_trunc(long);

//xの絶対値にyの符号を付加して返す copysign(x,y) (-255<=x,y<=255) 
long q23_copysign(long,long);

//剰余を計算する　mod(x,y) (-255<=x<=255,-255<=y<=255)
//ある整数nに対してr=x-n*yとなるrを返す。rはxと同符号で、rの絶対値はyの絶対値より小さい。
//yが0の場合、0を返す。
long q23_mod(long,long);

//平方根　sqrt(x)  (0≦x<256)
long q23_sqrt(unsigned long);

//立法根　cbrt(x)  (-255≦x<=255)
long q23_cbrt(long);

//自然対数関数 log(x) (0<x<255) 
long q23_log(long);

//2 を底とする対数関数 log2(x) (0<x<＝255) 
long q23_log2(long);

//常用対数関数 log10(x) (0<x<＝255) 
long q23_log10(long);

//指数関数 exp(x) (-255<=x<＝5.54) 
long q23_exp(long);

//sigmoid関数　 1/(1+exp(-x))　 (-255<=x<=255)
long q23_sigmoid(long);

//双曲線余弦関数 cosh(x) (-6.23<=x<＝6.23) 
long q23_cosh(long);

//双曲線正弦関数 sinh(x) (-6.23<=x<＝6.23)
long q23_sinh(long);

//双曲線正接関数 tanh(x) (-255<=x<＝255) 
long q23_tanh(long);

//双曲線逆余弦関数 arccosh(x) (1<=x<＝255) log(x + sqrt(x * x - 1.0))
long q23_acosh(long);

//双曲線逆正弦関数 arcsinh(x) (-255<=x<＝255) log(x + sqrt(x * x + 1.0))
long q23_asinh(long);

//双曲線逆正接関数 arctanh(x) (-0.99<=x<＝0.99) 0.5 * log((1.0 + x) / (1.0 - x))
long q23_atanh(long);

//余弦関数　 cos(x)　 (-256<=x<=256)
long q23_cos(long);

//正弦関数　 sin(x)　 (-255<=x<=255)
long q23_sin(long);

//正弦・余弦関数　 s=sin(x) c=cos(x)　 (-255<=x<=255)
void q23_sincos(long,long *,long *);

//正接関数　 tan(x)　 (-1.56<=x<=1.56) (0.498π<=x<=-0.498π)
long q23_tan(long);

//逆正接関数　 arctan(x)　 (-255<=x<=255)
long q23_atan(long);

//偏角を求める関数　 arctan2(x,y)　 (-255<=x,y<=255)
long q23_atan2(long,long);

//半径を求める関数　sqrt(a^2+b^2)  (-181≦x,y<=181) hypotenuse（斜辺）
long q23_hypot(long,long);

//逆正弦関数　 arcsin(x)　 (-1<=x<=1)
long q23_asin(long);

//逆余弦関数　 arccos(x)　 (-1<=x<=1)
long q23_acos(long);

//ｘのｙ乗　 pow(x)=exp(y*log(x)) 　0<x≦255，-255≦y≦255，-255≦y*log(x)≦5.54
long q23_pow(long,long);

//ガンマ関数の自然対数 　 (0<x<=76.9)
long q23_lgamma(long);

//ガンマ関数 　 (-0.98<=x<=-0.02,0.02<=x<=6.4)
long q23_tgamma(long);

//誤差関数 erf(x) (-255<=x<=255)
long  q23_erf(long);

//余誤差関数 1-erf(x) (-255<=x<=255)
long  q23_erfc(long);




//q26関数定義 

long q26_mul(long,long);


long q26_div(long,long);


//床関数　floor(x) (-31<=x<=31) 
long q26_floor(long);

//天井関数　ceil(x) (-31<=x<=31) 
long q26_ceil(long);

//四捨五入　round(x) (-31<=x<=31) 
long q26_round(long);

//x>yのときx-yを返し，x≦yのとき0を返す．　fdim(x,y) (-31<=x<=31) 
long q26_dim(long,long);

//2つの引数xとyの最大値を返す．　fmax(x,y) (-31<=x<=31) 
long q26_max(long,long);

//2つの引数xとyの最小値を返す．　fmin(x,y) (-31<=x<=31) 
long q26_min(long,long);

//整数値に丸める関数　trunc(x) (-31<=x<=31) 
//小数点以下を0方向に切り捨てて整数値を返す
long q26_trunc(long);

//xの絶対値にyの符号を付加して返す copysign(x,y) (-31<=x,y<=31) 
long q26_copysign(long,long);

//剰余を計算する　mod(x,y) (-31<=x<=31,-31<=y<=31)
//ある整数nに対してr=x-n*yとなるrを返す。rはxと同符号で、rの絶対値はyの絶対値より小さい。
//yが0の場合、0を返す。
long q26_mod(long,long);

//平方根　sqrt(x)  (0≦x<32)
long q26_sqrt(unsigned long);

//立法根　cbrt(x)  (-31≦x<=31)
long q26_cbrt(long);

//自然対数関数 log(x) (0<x<＝31) 
long q26_log(long);

//常用対数関数 log10(x) (0<x<＝31) 
long q26_log10(long);

//2 を底とする対数関数 log2(x) (0<x<＝31) 
long q26_log2(long);

//指数関数 exp(x) (-31<=x<＝3.46) 
long q26_exp(long);

//双曲線余弦関数 cosh(x) (-4.15<=x<＝4.15) 
long q26_cosh(long);

//双曲線正弦関数 sinh(x) (-4.15<=x<＝4.15)
long q26_sinh(long);

//双曲線正接関数 tanh(x) (-31<=x<＝31) 
long q26_tanh(long);

//双曲線逆余弦関数 arccosh(x) (1<=x<＝31) log(x + sqrt(x * x - 1.0))
long q26_acosh(long);

//双曲線逆正弦関数 arcsinh(x) (-31<=x<＝31) log(x + sqrt(x * x + 1.0))
long q26_asinh(long);

//双曲線逆正接関数 arctanh(x) (-0.99<=x<＝0.99) 0.5 * log((1.0 + x) / (1.0 - x))
long q26_atanh(long);

//sigmoid関数　 1/(1+exp(-x))　 (-31<=x<=31)
long q26_sigmoid(long);

//余弦関数　 cos(x)　 (-31<=x<=31)
long q26_cos(long);

//正弦関数　 sin(x)　 (-31<=x<=31)
long q26_sin(long);

//正弦・余弦関数　 s=sin(x) c=cos(x)　 (-31<=x<=31)
void q26_sincos(long,long *,long *);

//正接関数　 tan(x)　 (-1.5<=x<=1.5) (0.49π<=x<=-0.49π)
long q26_tan(long);

//逆正接関数　 arctan(x)　 (-31<=x<=31)
long q26_atan(long);

//偏角を求める関数　 arctan2(x,y)　 (-31<=x,y<=31)
long q26_atan2(long,long);

//半径を求める関数　sqrt(a^2+b^2)  (-22.6≦x,y<=22.6) hypotenuse（斜辺）
long q26_hypot(long,long);

//逆正弦関数　 arcsin(x)　 (-1<=x<=1)
long q26_asin(long);

//逆余弦関数　 arccos(x)　 (-1<=x<=1)
long q26_acos(long);

//ｘのｙ乗　 pow(x)=exp(y*log(x)) 　0<x≦255，-255≦y≦255，-255≦y*log(x)≦3.46
long q26_pow(long,long);

//ガンマ関数の自然対数 　 (0<x<=17.4)
long q26_lgamma(long);

//ガンマ関数 　 (-0.96<=x<=-0.04,0.04<=x<=5.1)
long q26_tgamma(long);

//誤差関数 erf(x) (-31<=x<=31)
long  q26_erf(long);

//余誤差関数 1-erf(x) (-31<=x<=31)
long  q26_erfc(long);




//q29関数定義 

long q29_mul(long,long);//注意　29bitと26bitの積は26bitになる 


long q29_div(long,long);

//平方根　sqrt(x)  (0≦x<4)
long q29_sqrt(unsigned long);

//立法根　cbrt(x)  (-4<x<4)
long q29_cbrt(long);

//自然対数関数 log(x) (0.019<x<4) 
long q29_log(long);

//指数関数 exp(x) (-4<x<＝1.38) 
long q29_exp(long);

//29bit余弦関数　 cos(x)　 (-3.1416<=x<=3.1416)
long q29_cos(long);

//29bit正弦関数　 sin(x)　 (-3.1416<=x<=3.1416)
long q29_sin(long);

//29bit正弦・余弦関数　 s=sin(x) c=cos(x)　 (-31<=x<=31)
void q29_sincos(long ,long *,long *);

//逆正接関数　 arctan(x)　 (-4<x<4)
long q29_atan(long);

//偏角を求める関数　 arctan2(x,y)　 (-4<x,y<4)
long q29_atan2(long,long);

//半径を求める関数　sqrt(a^2+b^2)  (-2.82≦x,y<=2.82) hypotenuse（斜辺）
long q29_hypot(long,long);



#define   QU_MUL(a,b,n)	((unsigned long)(((long long)(a) * (long long)(b)+(1<<((n)-1)))>>(n)))
#define   Q30U_MUL(a,b)	QU_MUL(a,b,30)
#define   Q31U_MUL(a,b)	QU_MUL(a,b,31)

long q23_mul(long a,long b)//注意　23bitと26bitの積は26bitになる 
{
	return (long)(((long long)a * (long long)b+Q_23/2)>>23);
}


long q26_mul(long a,long b)
{
	return (long)(((long long)a * (long long)b+Q_26/2)>>26);
}


long q29_mul(long a,long b)//注意　29bitと26bitの積は26bitになる 
{
	return (long)(((long long)a * (long long)b+Q_29/2)>>29);
}


long q23_div(long a,long b)
{
	if ((a >= 0 && b >= 0) || (a < 0 && b < 0))
        return (long)((((long long)a << 23) + (b >>1))/b);
    else
        return (long)((((long long)a << 23) - (b >>1))/b);
}


long q26_div(long a,long b)
{
	if ((a >= 0 && b >= 0) || (a < 0 && b < 0))
        return (long)((((long long)a << 26) + (b >>1))/b);
    else
        return (long)((((long long)a << 26) - (b >>1))/b);
}


long q29_div(long a,long b)
{
	if ((a >= 0 && b >= 0) || (a < 0 && b < 0))
        return (long)((((long long)a << 29) + (b >>1))/b);
    else
        return (long)((((long long)a << 29) - (b >>1))/b);
}


//床関数　floor(x) (-255<=x<=255) 
long q23_floor(long x){
	return x & Q23_FLOOR;
}


//床関数　floor(x) (-31<=x<=31) 
long q26_floor(long x){
	return x & Q26_FLOOR;
}


//天井関数　ceil(x) (-255<=x<=255) 
long q23_ceil(long x){
	return -((-x) & Q23_FLOOR); 
}


//天井関数　ceil(x) (-31<=x<=31) 
long q26_ceil(long x){
	return -((-x) & Q26_FLOOR); 
}


//四捨五入　round(x) (-255<=x<=255) 
long q23_round(long x){
	if (x>=0){
		return (x + Q_23/2) & Q23_FLOOR;
	}else{
		return (x + (Q_23/2-1)) & Q23_FLOOR;
	}
}


//四捨五入　round(x) (-31<=x<=31) 
long q26_round(long x){
	if (x>=0){
		return (x + Q_26/2) & Q26_FLOOR;
	}else{
		return (x + (Q_26/2-1)) & Q26_FLOOR;
	}
}


//x>yのときx-yを返し，x≦yのとき0を返す．　fdim(x,y) (-255<=x<=255) 
long q23_dim(long x,long y){
	if (x>y){
		return x-y;
	}else{
		return 0;
	}
}


//x>yのときx-yを返し，x≦yのとき0を返す．　fdim(x,y) (-31<=x<=31) 
long q26_dim(long x,long y){
	if (x>y){
		return x-y;
	}else{
		return 0;
	}
}


//2つの引数xとyの最大値を返す．　fmax(x,y) (-255<=x<=255) 
long q23_max(long x,long y){
	if (x>=y){
		return x;
	}else{
		return y;
	}
}


//2つの引数xとyの最大値を返す．　fmax(x,y) (-31<=x<=31) 
long q26_max(long x,long y){
	if (x>=y){
		return x;
	}else{
		return y;
	}
}


//2つの引数xとyの最小値を返す．　fmin(x,y) (-255<=x<=255) 
long q23_min(long x,long y){
	if (x<=y){
		return x;
	}else{
		return y;
	}
}


//2つの引数xとyの最小値を返す．　fmin(x,y) (-31<=x<=31) 
long q26_min(long x,long y){
	if (x<=y){
		return x;
	}else{
		return y;
	}
}


//整数値に丸める関数　trunc(x) (-255<=x<=255) 
// 引数を最も近い整数値に丸める (ただし，その絶対値が引数の絶対値より大きくない値)
long q23_trunc(long x){
	if (x>=0){
		return x & Q23_FLOOR;
	}else{
		return -((-x ) & Q23_FLOOR);
	}
}


//整数値に丸める関数　trunc(x) (-31<=x<=31) 
// 引数を最も近い整数値に丸める (ただし，その絶対値が引数の絶対値より大きくない値)
long q26_trunc(long x){
	if (x>=0){
		return x & Q26_FLOOR;
	}else{
		return -((-x ) & Q26_FLOOR);
	}
}


//xの絶対値にyの符号を付加して返す copysign(x,y) (-255<=x,y<=255) 
long q23_copysign(long x,long y){
	if (x<0) x=-x;
	if (y<0) x=-x;
	return x;
}


//xの絶対値にyの符号を付加して返す copysign(x,y) (-31<=x,y<=31) 
long q26_copysign(long x,long y){
	if (x<0) x=-x;
	if (y<0) x=-x;
	return x;
}

//剰余を計算する　mod(x,y) (-255<=x<=255,-255<=y<=255)
//ある整数nに対してr=x-n*yとなるrを返す。rはxと同符号で、rの絶対値はyの絶対値より小さい。
//yが0の場合、0を返す。
long q23_mod(long x,long y){
	if (y==0) return 0;//最小値 
	long z,q,r;
	int flag=x<0;
	if (flag)x=-x;
	if (y<0)y=-y;
	z=Q23_PDIV(x,y);
	q=z & Q23_FLOOR;//q=Q23_floor(z);
	r=x-q23_mul(y,q);
	if (flag)r=-r;
	return r;
}

//剰余を計算する　mod(x,y) (-31<=x<=31,-31<=y<=31)
//ある整数nに対してr=x-n*yとなるrを返す。rはxと同符号で、rの絶対値はyの絶対値より小さい。
//yが0の場合、0を返す。
long q26_mod(long x,long y){
	if (y==0) return 0;//最小値 
	long z,q,r;
	int flag=x<0;
	if (flag)x=-x;
	if (y<0)y=-y;
	z=Q26_PDIV(x,y);
	q=z & Q26_FLOOR;//q=Q26_floor(z);
	r=x-q26_mul(y,q);
	if (flag)r=-r;
	return r;
}


//平方根　sqrt(x)  (1≦x<4)
unsigned long q30u_sqrt(unsigned long a)
{
    unsigned long x,y,h;
	int k;

	static unsigned long kugiri[7]={1273712137UL,1514707535UL,1801300978UL,2142119940UL,2547424273UL,3029415071UL,3602601957UL};
	static unsigned long s[8]={1542328040UL,1414321049UL,1296938120UL,1189297500UL,1090590616UL,1000076004UL,917073739UL,840960327UL};
	static unsigned long t[8]={471440350UL,363530205UL,280320109UL,216156353UL,166679334UL,128527337UL,99108125UL,76422812UL};

	for (k=7;k>0;k--) if (a>kugiri[k-1])break;
	x= s[k]-Q30U_MUL(t[k],a);

	y=Q30U_MUL(a,x);
	y=Q30U_MUL(x,y);
	if (y<=Q_30)h=Q_30-y;else h=0;
	
	//x = x *+x* h * (8 + h * (6 + 5 * h)) / 16
	y=Q_27*3+((h*5)>>4);
	y=Q_29+Q30U_MUL(h,y);
	y=Q30U_MUL(h,y);
	x=x+Q30U_MUL(x,y);
	return Q30U_MUL(a,x);
}


//平方根　sqrt(x)  (1/2≦x<2)
unsigned long q31u_sqrt(unsigned long a)
{
    unsigned long x,y,h;
	int k;

	static unsigned long kugiri[7]={1273712137UL,1514707535UL,1801300978UL,2142119940UL,2547424273UL,3029415071UL,3602601957UL};
	static unsigned long s[8]={3028927791UL,2972085324UL,2875429958UL,2752458815UL,2613215730UL,2465111892UL,2313556131UL,2162438646UL};
	static unsigned long t[8]={2666869345UL,2056437387UL,1585730000UL,1222764986UL,942880699UL,727060411UL,560640218UL,432312707UL};

	for (k=7;k>0;k--) if (a>kugiri[k-1])break;
	x= s[k]-Q31U_MUL(t[k],(a-Q_30));

	y=Q31U_MUL(a,x);
	y=Q31U_MUL(x,y);
	if (y<=Q_31)h=Q_31-y;else h=0;
	
	//x = x + x* h * (8 + h * (6 + 5 * h)) / 16
	y=Q_28*3+((h*5)>>4);
	y=Q_30+Q31U_MUL(h,y);
	y=Q31U_MUL(h,y);
	x=x+Q31U_MUL(x,y);
	return Q31U_MUL(a,x);
}


int sub_sqrt(unsigned long *a)
{
	int n=0;
    while (*a<Q_30){
    	*a=*a<<2;
    	n=n+1;
	}
	return n;
}


//平方根　sqrt(x)  (0≦x<256)
long q23_sqrt(unsigned long a)
{
	if (a<=0)return 0;
	int n=sub_sqrt(&a);
	unsigned long x=q31u_sqrt(a);
	return (long)((x+(1<<(n+3)))>>(n+4));
}


//平方根　sqrt(x)  (0≦x<32)
long q26_sqrt(unsigned long a)
{
	if (a<=0)return 0;
	int n=sub_sqrt(&a);
	unsigned long x=q30u_sqrt(a);
	return (long)((x+(1<<(n+1)))>>(n+2));
}


//平方根　sqrt(x)  (0≦x<4)
long q29_sqrt(unsigned long a)
{
	if (a<=0)return 0;
	int n=sub_sqrt(&a);
	unsigned long x=q31u_sqrt(a);
	return (long)((x+(1<<n))>>(n+1));
}


//立法根サブ　cbrt(x)
long q29sub_cbrt(long a,int bit)
{
	if (a==0)return 0;
	int flag=(a<0);
	if (flag)a=-a;
    long x,y,z;
	int k,n=0;
    while (a<Q_29/3){
    	a=a<<3;
    	n=n+1;
	}

	static long kugiri[5]={272574524 , 422993569 , 656420697 , 1018663554 , 1580808527};
	static long s[6]={1607619602 , 1199363861 , 894784853 , 667553826 , 498028223 , 371553725};
	static long t[6]={1548603684 , 744490999 , 357913941 , 172067076 , 82721222 , 39768216};

	for (k=5;k>0;k--) if (a>kugiri[k-1])break;
	x= s[k]-q29_mul(t[k],a);
	
    for(k=2;;){
		y=q29_mul(a,x);
		y=q29_mul(x,y);
		y=q29_mul(y,y);
		if (k==0)break;
		k--;
		x=x+(x-y)/3;		
	};
	x=x+(x-y+1)/3;		
	x=q29_mul(a,x);

	if (bit==23) x=(x+(1<<(n+3)))>>(n+4);
	else if(bit==26) x=(x+(1<<(n+1)))>>(n+2);
	else if (n>0) x=(x+(1<<(n-1)))>>n;//bit=29

    if (flag) x=-x;
	return x;	 
}


//立法根　cbrt(x)  (-255≦x<=255)
long q23_cbrt(long a)
{
	return q29sub_cbrt(a,23);	 
}


//立法根　cbrt(x)  (-31≦x<=31)
long q26_cbrt(long a)
{
	return q29sub_cbrt(a,26);	 
}


//立法根　cbrt(x)  (-4<x<4)
long q29_cbrt(long a)
{
	return q29sub_cbrt(a,29);	 
}


//自然対数関数 log(x) (1<=x<4) 
long q29sub_log(long x,int *n)
{
    static long table[8]={
		404678155 ,// log( 17 /8)
 		464391963 ,// log( 19 /8)
 		518123861 ,// log( 21 /8)
 		566963962 ,// log( 23 /8)
 		611729123 ,// log( 25 /8)
 		653047267 ,// log( 27 /8)
 		691411506 ,// log( 29 /8)
 		727216165 ,// log( 31 /8)
    };
	
    long y,h,k,z;
    *n=0;

    while (x<Q_30){
    	x=x<<1;
    	*n=*n+1;
	}			

    y=(x & 0xF8000000)+Q_29/8;//2<=x<4 上位5bit 
    unsigned long xpy=(unsigned long)x+(unsigned long)y;
    if (x>=y)
    	k=Q29_PDIV((x-y)<<1,xpy);
	else
    	k=-Q29_PDIV((y-x)<<1,xpy);
    h=k>>1;
    z=q29_mul(h,h);
    static long waru[2]={178956971,Q_29};//{Q_29/3,Q_29}
    long *p=waru;
    y=107374182; //Q_29/5
    do{
        y=q29_mul(y,z)+(*p);
        p++;
    }while (p<waru+2);
    y=q29_mul(y,k);
    
    return y+table[(x & 0x38000000)>>27]; //0から7  
}



//自然対数関数 log(x) (0<x<255) 
long q23_log(long x)
{
	if (x<=0) return  -Q_MAX;//最小値 
	
    long y;
    int n;
    y=q29sub_log(x,&n);
    return ((y+32)>>6)-(n-6)*Q23_LN2;//Q23_LN2=log(2)
}


//自然対数関数 log(x) (0<x<＝31) 
long q26_log(long x)
{
	if (x<=0) return  -Q_MAX;//最小値 
	
    long y;
    int n;
    y=q29sub_log(x,&n);
    return ((y+4)>>3)-(n-3)*Q26_LN2;//+((n+1)>>3);//Q26_LN2=log(2) ((n+1)>>3);
}


//自然対数関数 log(x) (0.019<x<4) 
long q29_log(long x)
{
	if (x<=0) return  -Q_MAX;//最小値 
	
    long y;
    int n;
    y=q29sub_log(x,&n);
    return y-n*Q29_LN2;//Q29_LN2=log(2)
}


//対数関数用サブ　 log(x) (0<x<＝31) Q_26を入力しQ_29を出力する 
long q26in_q29out_log(long x)
{
	if (x<=0) return  -Q_MAX;//最小値 
	
    long y;
    int n;
    y=q29sub_log(x,&n);
    return y-(n-3)*Q29_LN2;//Q29_LN2=log(2)
}


//常用対数関数 log10(x) (0<x<＝255) 
long q23_log10(long x)
{
	if (x>=Q_23/32) return ((q26_mul(q26_log(x),Q26_LOG10E)+4)>>3)+7575668;//log10(8)(Q_23)
	if (x>0) return q23_mul(q23_log(x),Q23_LOG10E);
	return  -Q_MAX;//最小値
}


//常用対数関数 log10(x) (0<x<＝31) 
long q26_log10(long x)
{
	if (x>=Q_26/32) return (q29_mul(q26in_q29out_log(x),466320149)+4)>>4;//log(x)/log(10)
	if (x>0) return q26_mul(q26_log(x),Q26_LOG10E);
	return  -Q_MAX;//最小値
}


//2 を底とする対数関数 log2(x) (0<x<＝255) 
long q23_log2(long x)
{
	if (x>=Q_23/32) return ((q26_mul(q26_log(x),Q26_LOG2E)+4)>>3)+Q_23*3;
	if (x>0) return q23_mul(q23_log(x),Q23_LOG2E);
	return  -Q_MAX;//最小値
}


//2 を底とする対数関数 log2(x) (0<x<＝31) 
long q26_log2(long x)
{
	if (x>=Q_26/32) return (q29_mul(q26in_q29out_log(x),387270501)+1)>>2;//log(x)/log(2)
	if (x>0) return q26_mul(q26_log(x),Q26_LOG2E);
	return  -Q_MAX;//最小値
}



//指数関数 exp(x) (-log(2)/2<=x<log(2)/2) 
long q30sub_exp(long x)
{
    static long waru[8]={
 		213044 ,//Q_30/ 7 !
 		1491308 ,//Q_30/ 6 !
 		8947849 ,//Q_30/ 5 !
 		44739243 ,//Q_30/ 4 !
 		178956971 ,//Q_30/ 3 !
 		536870912 ,//Q_30/ 2 !
 		1073741824 ,//Q_30/ 1 !
		Q_30
	};
    long *p=waru;	
    long y=26631; //Q_30/ 8 !
    do{
        y=Q30_MUL(y,x)+(*p);
        p++;
    }while (p<waru+8);
    return y;
}


//指数関数 exp(x) (-255<=x<＝5.54) 
long q23_exp(long x)
{
	if (x>46516319)return Q_MAX;//最大値
	if (x<=-16*Q_23) return 0;
    int n=0;
    while (x<-Q23_LN2*6){
    	x=x+Q23_LN2*6;
    	n=n-6;}
    while (x<-Q23_LN2/2){
    	x=x+Q23_LN2;
    	n--;}
    while (x>Q23_LN2/2){
    	x=x-Q23_LN2;
    	n++;}
    x=x*128;
    long y=q30sub_exp(x);
    if (n>=7) y=y<<(n-7);else y=(y+(1<<(6-n)))>>(7-n);
    return y;
}


//指数関数 exp(x) (-31<=x<＝3.46) 
long q26_exp(long x)
{
	if (x>232581599)return Q_MAX;//最大値
	if (x<=-19*Q_26) return 0;
    int n=0;
    while (x<-Q26_LN2*6){
    	x=x+Q26_LN2*6;
    	n=n-6;}
    while (x<-Q26_LN2/2){
    	x=x+Q26_LN2;
    	n--;}
    while (x>Q26_LN2/2){
    	x=x-Q26_LN2;
    	n++;}
    x=x*16;
    long y=q30sub_exp(x);
    if (n>=4) y=y<<(n-4);else y=(y+(1<<(3-n)))>>(4-n);
    return y;
}


//指数関数 exp(x) (-4<x<＝1.38) 
long q29_exp(long x)
{
	if (x>744261117)return Q_MAX;//最大値
    int n=0;
    while (x<-Q29_LN2/2){
    	x=x+Q29_LN2;
    	n--;}
    while (x>Q29_LN2/2){
    	x=x-Q29_LN2;
    	n++;}
    x=x*2;
    long y=q30sub_exp(x);
    if (n>=1) y=y<<(n-1);else y=(y+(1<<(-n)))>>(1-n);
    return y;
}


//sigmoid関数　 1/(1+exp(-x))　 (-255<=x<=255)
long q23_sigmoid(long x)
{
	if (x>=0) return Q23_PDIV(Q_23,Q_23+q23_exp(-x));
	else return Q_23-Q23_PDIV(Q_23,Q_23+q23_exp(x));
}


//sigmoid関数　 1/(1+exp(-x))　 (-31<=x<=31)
long q26_sigmoid(long x)
{
	if (x>=0) return Q26_PDIV(Q_26,Q_26+q26_exp(-x));
	else return Q_26-Q26_PDIV(Q_26,Q_26+q26_exp(x));
}


//双曲線余弦関数 cosh(x) (-6.23<=x<＝6.23) 
long q23_cosh(long x)
{
	long y;
	if (x<0) x=-x;
	if (x>52330827) return Q_MAX;//最大値
	y=q23_exp(x-Q23_LN2);
	return y+Q23_PDIV(Q_23/4,y);
}


//双曲線余弦関数 cosh(x) (-4.15<=x<＝4.15) 
long q26_cosh(long x)
{
	long y;
	if (x<0) x=-x;
	if (x>279081529) return Q_MAX;//最大値
	y=q26_exp(x-Q26_LN2);
	return y+Q26_PDIV(Q_26/4,y);
}


//双曲線正弦関数 sinh(x) (-6.23<=x<＝6.23)
long q23_sinh(long x)
{
	long y,z,w;
	if (x>=0)y=x;else y=-x;
	if (y>52330859){
		if (y>0)return Q_MAX;//最大値
		else return  -Q_MAX;//最小値
	}
	w=q23_exp(y-Q23_LN2);
	z=w-Q23_PDIV(Q_23/4,w);
	if (x>=0)return z;else return -z;	
}


//双曲線正弦関数 sinh(x) (-4.15<=x<＝4.15)
long q26_sinh(long x)
{
	long y,z,w;
	if (x>=0)y=x;else y=-x;
	if (y>279097919){
		if (y>0)return Q_MAX;//最大値
		else return  -Q_MAX;//最小値
	}
	w=q26_exp(y-Q26_LN2);
	z=w-Q26_PDIV(Q_26/4,w);
	if (x>=0)return z;else return -z;	
}


//双曲線正接関数 tanh(x) (-255<=x<＝255) 
long q23_tanh(long x)
{
	if (x>10*Q_23) return Q_23;
	if (x<-10*Q_23) return -Q_23;
	long z;
	if (x>0){
		z=q23_exp(-2*x);	
		return Q23_PDIV(Q_23-z,Q_23+z);		
	}else{
		z=q23_exp(2*x);	
		return -Q23_PDIV(Q_23-z,Q_23+z);
	}
}


//双曲線正接関数 tanh(x) (-31<=x<＝31) 
long q26_tanh(long x)
{
	if (x>10*Q_26) return Q_26;
	if (x<-10*Q_26) return -Q_26;
	long z;
	if (x>0){
		z=q26_exp(-2*x);	
		return Q26_PDIV(Q_26-z,Q_26+z);		
	}else{
		z=q26_exp(2*x);	
		return -Q26_PDIV(Q_26-z,Q_26+z);
	}
}


//双曲線逆余弦関数 arccosh(x) (1<=x<＝255) log(x + sqrt(x * x - 1.0))
long q23_acosh(long x)
{
	if (x<=Q_23*4){
		return q23_log(x+q23_sqrt(q23_mul(x,x)-Q_23));
	}else{
		long z=Q23_PDIV(Q_23,x);
		return q23_log(x)+q23_log(Q_23+q23_sqrt(Q_23-q23_mul(z,z)));
	}
}


//双曲線逆余弦関数 arccosh(x) (1<=x<＝31) log(x + sqrt(x * x - 1.0))
long q26_acosh(long x)
{
	if (x<=Q_26*4){
		return q26_log(x+q26_sqrt(q26_mul(x,x)-Q_26));
	}else{
		long z=Q26_PDIV(Q_26,x);
		return q26_log(x)+q26_log(Q_26+q26_sqrt(Q_26-q26_mul(z,z)));
	}
}


//双曲線逆正弦関数 arcsinh(x) (-255<=x<＝255) log(x + sqrt(x * x + 1.0))
long q23_asinh(long x)
{
	long y,z,w;
	if (x>=0)y=x; else y=-x;
	if (y<=Q_23*4){
		w=q23_log(y+q23_sqrt(q23_mul(y,y)+Q_23));
	}else{
		z=Q23_PDIV(Q_23,y);
		w=q23_log(y)+q23_log(Q_23+q23_sqrt(Q_23+q23_mul(z,z)));
	}
	if (x<0)w=-w;
	return w;
}


//双曲線逆正弦関数 arcsinh(x) (-31<=x<＝31) log(x + sqrt(x * x + 1.0))
long q26_asinh(long x)
{
	long y,z,w;
	if (x>=0)y=x; else y=-x;
	if (y<=Q_26*4){
		w=q26_log(y+q26_sqrt(q26_mul(y,y)+Q_26));
	}else{
		z=Q26_PDIV(Q_26,y);
		w=q26_log(y)+q26_log(Q_26+q26_sqrt(Q_26+q26_mul(z,z)));
	}
	if (x<0)w=-w;
	return w;
}


//双曲線逆正接関数 arctanh(x) (-0.99<=x<＝0.99) 0.5 * log((1.0 + x) / (1.0 - x))
long q23_atanh(long x)
{
	if (x<=-Q_23) return  -Q_MAX;//最小値 
	if (x>= Q_23) return  Q_MAX;//最大値 
	return ( q23_log(Q_23 + x) - q23_log(Q_23 - x))>>1;
}


//双曲線逆正接関数 arctanh(x) (-0.99<=x<＝0.99) 0.5 * log((1.0 + x) / (1.0 - x))
long q26_atanh(long x)
{
	if (x<=-Q_26) return  -Q_MAX;//最小値 
	if (x>= Q_26) return  Q_MAX;//最大値 
	return ( q26_log(Q_26 + x) - q26_log(Q_26 - x))>>1;
}


//30bit余弦関数　 cos(x)　 (-π/2<=x<=π/2)
long q30sub_cos(long x)
{
	static long w[3]={		
 		2982616,//Q_30/ 6 !*2
 		-89478485,//-Q_30/ 4 !*2
 		Q_30//Q_30/2*2;
 	};	
    long xx,kou,y;
    long *p=w;
	kou=Q30_MUL(x>>1,x);
	xx = (kou+4)>>3;
	y=-53261;//-Q_30/ 8 !*2;
	do{
	    y=Q30_MUL(y,xx)+(*p);
		p++;
	}while (p<w+3);
	y = Q30_MUL(kou,y);
    y = Q30_MUL(y,Q_30-((y+16)>>5));
    y = Q30_MUL(y,Q_30-((y+4)>>3));
    y = Q_30-y;
	return y;	 
}


//余弦関数　 cos(x)　 (-256<=x<=256)
long q23_cos(long x)
{
    int n=0;
    long y;
	if (x<0) x=-x;

    while (x>Q23_PI*10){
    	x=x-Q23_PI*10;
		n=n+10;
	}
    while (x>Q23_PI_2){
    	x=x-Q23_PI;
		n++;
	}
	x=x-((n+2)>>2);
	if (n>=36) x = x-1;

	x=x<<7;	
	y=q30sub_cos(x);	
    y = (y+64)>>7;

    if (n&0x1) y=-y;
	return y;	 
}


//余弦関数　 cos(x)　 (-31<=x<=31)
long q26_cos(long x)
{
    int n=0;
    long y;
	if (x<0) x=-x;

    while (x>Q26_PI_2){
    	x=x-Q26_PI;
		n++;
	}
	x=x-((n+4)>>3);	

	x=x<<4;
	y=q30sub_cos(x);
    y = (y+8)>>4;

    if (n&0x1) y=-y;
	return y;
}


//29bit余弦関数　 cos(x)　 (-3.1416<=x<=3.1416)
long q29_cos(long x)
{
    int flag=0;
    long y;
	if (x<0) x=-x;
    if (x>=Q29_PI_2){
		x = x-Q29_PI;
    	flag=1;
	}
	x=x<<1;
	y=q30sub_cos(x)>>1;

    if (flag) y=-y;
	return y;	 
}



//正弦関数　 sin(x)　 (-255<=x<=255)
long q23_sin(long x)
{
	if (x>=0) {
		x=x-Q23_PI_2;
		return q23_cos(x);
	}
	else{
		x=x+Q23_PI_2;
		return -q23_cos(x);
	} 	
}


//正弦関数　 sin(x)　 (-31<=x<=31)
long q26_sin(long x)
{
	if (x>=0) {
		x=x-Q26_PI_2;
		return q26_cos(x);
	}
	else{
		x=x+Q26_PI_2;
		return -q26_cos(x);
	} 	
}


//29bit正弦関数　 sin(x)　 (-3.1416<=x<=3.1416)
long q29_sin(long x)
{
    int flag=0;
    long y;
    if (x<0) {x=-x; flag=1-flag;}
    x=x-843314856;//x=x-π/2 
    if (x>843314856){
    	x=x-1686629713;//x=x-π
    	flag=1-flag;
	}
	x=x*2-1;
    y=q30sub_cos(x);
    if (flag) y=-y;
	return y>>1;	 
}


//30bit正弦・余弦関数　 s=sin(x) c=cos(x)　 (-π/2<=x<=π/2)
void q30sub_sincos(long x,long *s,long *c)
{
	//sk[4]={-Q_30/(2*3*4*5*6*7)-0.5,Q_30/(2*3*4*5)+0.5,-Q_30/(2*3)-0.5,Q_30}
	//ck[4]={-Q_30/(3*4*5*6*7*8)-0.5,Q_30/(3*4*5*6)+0.5,-Q_30/(3*4)-0.5,Q_30}	
	static long sk[3]={8947849,-178956971,Q_30};
	static long ck[3]={2982616,-89478485,Q_30};
    long xx,kou;
    long *sp=sk,*cp=ck;
	kou=Q30_MUL(x>>1,x);
	xx = (kou+4)>>3;
	*s=Q30_MUL(2959,xx)-213044;
	*c=-53261;
	do{
	    *s=Q30_MUL(*s,xx)+(*sp);
	    *c=Q30_MUL(*c,xx)+(*cp);
		sp++;
		cp++;
	}while (sp<sk+3);
	*s=Q30_MUL(x,*s);
	*c=Q30_MUL(kou,*c);

	int i;
    *s= Q30_MUL(*s,Q_30-((*c+8)>>4));
    *c= Q30_MUL(*c,Q_30-((*c+16)>>5));
    *s= Q30_MUL(*s,Q_30-((*c+1)>>2));
    *c= Q30_MUL(*c,Q_30-((*c+4)>>3));
	*c=Q_30-*c;
}


//正弦・余弦関数　 s=sin(x) c=cos(x)　 (-255<=x<=255)
void q23_sincos(long x,long *s,long *c)
{
    int n=0,flag=0;

	if (x<0){x=-x;flag=1;}
    while (x>Q23_PI*10){
    	x=x-Q23_PI*10;
		n=n+10;
	}
    while (x>Q23_PI_2){
    	x=x-Q23_PI;
		n++;
	}
	x=x-((n+2)>>2);
	if (n>=36) x = x-1;

    x=x<<7;	
	q30sub_sincos(x,s,c);
	*s=(*s+64)>>7;
	*c=(*c+64)>>7;

	if (flag) *s=-*s;
	if (n&0x1) {
		*s=-*s;
		*c=-*c;
	}	 
}


//正弦・余弦関数　 s=sin(x) c=cos(x)　 (-31<=x<=31)
void q26_sincos(long x,long *s,long *c)
{
    int n=0,flag=0;

	if (x<0){x=-x;flag=1;}
    while (x>Q26_PI_2){
    	x=x-Q26_PI;
		n++;
	}
	x=x-((n+4)>>3);

    x=x<<4;
	q30sub_sincos(x,s,c);
	*s=(*s+8)>>4;
	*c=(*c+8)>>4;

	if (flag) *s=-*s;
	if (n&0x1) {
		*s=-*s;
		*c=-*c;
	}	 
}


//29bit正弦・余弦関数　 s=sin(x) c=cos(x)　 (-3.9<=x<=3.9)
void q29_sincos(long x,long *s,long *c)
{
    int n=0,flag=0;

	if (x<0){x=-x; flag=1;}
    if (x>Q29_PI_2){
    	n = 1;
    	x = x-Q29_PI;
	}
	
    x=x<<1;
	q30sub_sincos(x,s,c);
	*s=*s>>1;
	*c=*c>>1;
	
	if (flag) *s=-*s;
	if (n) {
		*s=-*s;
		*c=-*c;
	}	 
}


//正接関数　 tan(x)　 (-1.56<=x<=1.56) (0.498π<=x<=-0.498π)
long q23_tan(long x)
{
	long s,c;
	q23_sincos(x,&s,&c);
	if (-32767<c && c<32767){
		if ((s > 0 && c >= 0) || (s < 0 && c <= 0))
			return Q_MAX;
		else 
			return -Q_MAX;
	}
	return q23_div(s,c);		 
}


//正接関数　 tan(x)　 (-1.5<=x<=1.5) (0.49π<=x<=-0.49π)
long q26_tan(long x)
{
	long s,c;
	q26_sincos(x,&s,&c);
	if (-2096128<c && c<2096128){
		if ((s > 0 && c >= 0) || (s < 0 && c <= 0))
			return Q_MAX;
		else 
			return -Q_MAX;
	}
	return q26_div(s,c);		 
}


long q30sub_atan(long x)//0<=x<=1
{
 	static long atn0[16]={
 		33543516 ,//Q_30*arctan( 0 /16+1/32)
 		100369930 ,//Q_30*arctan( 1 /16+1/32)
 		166426484 ,//Q_30*arctan( 2 /16+1/32)
 		231238569 ,//Q_30*arctan( 3 /16+1/32)
 		294385059 ,//Q_30*arctan( 4 /16+1/32)
 		355511705 ,//Q_30*arctan( 5 /16+1/32)
 		414338361 ,//Q_30*arctan( 6 /16+1/32)
 		470660297 ,//Q_30*arctan( 7 /16+1/32)
 		524344587 ,//Q_30*arctan( 8 /16+1/32)
 		575322936 ,//Q_30*arctan( 9 /16+1/32)
 		623582386 ,//Q_30*arctan( 10 /16+1/32)
 		669155185 ,//Q_30*arctan( 11 /16+1/32)
 		712108791 ,//Q_30*arctan( 12 /16+1/32)
 		752536690 ,//Q_30*arctan( 13 /16+1/32)
 		790550395 ,//Q_30*arctan( 14 /16+1/32)
 		826272767 //Q_30*arctan( 15 /16+1/32)
 	};
	static long w[2]={-357913941,Q_30};//{-Q_30/3,Q_30};
    long y,z,zz;
    long *p=w;
    if (x>=Q_30)return 843314857;//Q30_PI_4
	y=(x & (0xF<<26))+(Q_30>>5);
	z=Q_30+Q30_MUL(x,y);//z=(x-y)/(1+x*y);
    z=(long)(((long long)(x-y) << 30) /z);	//z=Q30_div(x-y,z);四捨五入を省略
	zz=Q30_MUL(z,z);
	y=214748365;//Q_30/5
	do{
	    y=Q30_MUL(y,zz)+(*p);
		p++;
	}while (p<w+2);
	return Q30_MUL(z,y)+atn0[x>>26];	 
}


#define Q30_PI_2 1686629713

//逆正接関数　 arctan(x)　 (-255<=x<=255)
long q23_atan(long x)
{
    long y;
    if (x>=0) y=x;else y=-x;

	if (y<=Q_23){
		y=(q30sub_atan(y<<7)+64)>>7;}
	else
		y=(Q30_PI_2-(q30sub_atan(Q23_PDIV(Q_30,y)))+64)>>7;
	if (x<0) y=-y;
	return y;	
}


//逆正接関数　 arctan(x)　 (-31<=x<=31)
long q26_atan(long x)
{
    long y;
    if (x>=0) y=x;else y=-x;

	if (y<=Q_26){
		y=(q30sub_atan(y<<4)+8)>>4;}
	else
		y=(Q30_PI_2-(q30sub_atan(Q26_PDIV(Q_30,y)))+8)>>4;
	if (x<0) y=-y;
	return y;	
}


//逆正接関数　 arctan(x)　 (-3.9<=x<=3.9)
long q29_atan(long x)
{
    long y;
    if (x>=0) y=x;else y=-x;

	if (y<=Q_29){
		y=q30sub_atan(y<<1)>>1;}
	else
		y=(Q30_PI_2-(q30sub_atan(Q29_PDIV(Q_30,y))))>>1;
	if (x<0) y=-y;
	return y;	
}


long q29sub_atan2(long y,long x)
{
    if(x>=y){
		return q30sub_atan(Q30_PDIV(y,x))>>1;
    }else{
		return (Q30_PI_2-q30sub_atan(Q30_PDIV(x,y)))>>1;
	}	 
}

#undef Q30_PI_2


//偏角を求める関数　 arctan2(x,y)　 (-4<x,y<4)
long q29_atan2(long y,long x)
{
	if (y>=0) {
		if (x>=0){
		    if (y==0 && x==0) return 0;
			return q29sub_atan2(y,x);}
		else
			return Q29_PI-q29sub_atan2(y,-x);
	} else {
		if (x>=0)
			return -q29sub_atan2(-y,x);
		else
			return -Q29_PI+q29sub_atan2(-y,-x);
	} 
}


//偏角を求める関数　 arctan2(x,y)　 (-31<=x,y<=31)
long q26_atan2(long y,long x)
{
    return (q29_atan2(y,x)+4)>>3;	 
}

//偏角を求める関数　 arctan2(x,y)　 (-255<=x,y<=255)
long q23_atan2(long y,long x)
{
	return (q29_atan2(y,x)+32)>>6;	 
}


//半径を求める関数　sqrt(a^2+b^2)  (-2.82≦x,y<=2.82) hypotenuse（斜辺）
long q29_hypot(long a,long b)
{
	int n=0;
	unsigned long x,y,c;

	if (a>=0) x=(unsigned long)a;else x=(unsigned long)(-a);
    if (b>=0) y=(unsigned long)b;else y=(unsigned long)(-b);
    if (x<y) {
		unsigned long tmp=x;
		x=y;
		y=tmp;
    }
    if (y==0) return (long)x;
    if (x<Q_30){
    	unsigned long s=(x<<1)+y;
		while(s<Q_31){
    		s=s<<1;
    		n++;		
		}
		x=x<<n;
		y=y<<n;	
	}
	c=Q30U_MUL(x,x)+Q30U_MUL(y,y);
	if (c<Q_30){ //1<=c<4に収める 
		c=c<<2;
		n++;	
	}

	x=q30u_sqrt(c);
	return (long)((n>0) ? ((x+(1UL<<(n-1)))>>n) : x);   	
}


//半径を求める関数　sqrt(a^2+b^2)  (-181≦x,y<=181) hypotenuse（斜辺）
long q23_hypot(long x,long y)
{
    return q29_hypot(x,y);
}


//半径を求める関数　sqrt(a^2+b^2)  (-22.6≦x,y<=22.6) hypotenuse（斜辺）
long q26_hypot(long x,long y)
{
    return q29_hypot(x,y);
}


//逆正弦関数　 arcsin(x)　 (-1<=x<=1)
long q23_asin(long x)
{
	if (x>Q_23/1024*1023)
		return Q23_PI_2-q23_sqrt(6*Q_23-2*q23_sqrt( 6*x+3*Q_23));//Pi/2-Sqrt(6-2Sqrt(6x+3))
	else if(x<-Q_23/1024*1023)
		return -Q23_PI_2+q23_sqrt(6*Q_23-2*q23_sqrt(-6*x+3*Q_23));//-Pi/2+Sqrt(6-2Sqrt(-6x+3))
	else
		return q23_atan(q23_div(x,q23_sqrt(Q_23-q23_mul(x,x))));	 
}


//逆正弦関数　 arcsin(x)　 (-1<=x<=1)
long q26_asin(long x)
{
	if (x>Q_26/1024*1023)
		return Q26_PI_2-q26_sqrt(6*Q_26-2*q26_sqrt( 6*x+3*Q_26));//Pi/2-Sqrt(6-2Sqrt(6x+3))
	else if(x<-Q_26/1024*1023)
		return -Q26_PI_2+q26_sqrt(6*Q_26-2*q26_sqrt(-6*x+3*Q_26));//-Pi/2+Sqrt(6-2Sqrt(-6x+3))
	else
		return q26_atan(q26_div(x,q26_sqrt(Q_26-q26_mul(x,x))));	 
}


//逆余弦関数　 arccos(x)　 (-1<=x<=1)
long q23_acos(long x){
	return Q23_PI_2-q23_asin(x);
}


//逆余弦関数　 arccos(x)　 (-1<=x<=1)
long q26_acos(long x){
	return Q26_PI_2-q26_asin(x);
}


//ｘのｙ乗　 pow(x)=exp(y*log(x)) 　0<x≦255，-255≦y≦255，-255≦y*log(x)≦5.54
long q23_pow(long x,long y){
	return q23_exp(q23_mul(y,q23_log(x)));
}


//ｘのｙ乗　 pow(x)=exp(y*log(x)) 　0<x≦255，-255≦y≦255，-255≦y*log(x)≦3.46
long q26_pow(long x,long y){
	return q26_exp(q26_mul(y,q26_log(x)));
}


//ガンマ関数の自然対数 　 (0<x<=76.9)
long q23_lgamma(long x)  
{
	long  w ,z,zurasi=0 ;
	if (x < Q_23*3){
		long v=x;
		x=x+Q_23;
		while (x < Q_23*3) {
			v=q23_mul(v,x);
			x=x+Q_23;
		}
		zurasi=q23_log(v);
	}
	long  x1=Q23_PDIV(Q_23,x);
 	static long k[5]={
 		7061,  		// ( 5.0 / 66.0) / (10 *  9);
 		-4993,  	// (-1.0 / 30.0) / ( 8 *  7);
 		6658 ,  	// ( 1.0 / 42.0) / ( 6 *  5);
 		-23302, 	// (-1.0 / 30.0) / ( 4 *  3);
 		699051 		// ( 1.0 / 6.0)  / ( 2 *  1);
	};
	long *p=k;
	w=q23_mul(x1,x1);
	z=-16085; // (-691.0 / 2730.0) / (12 * 11);
	do{
	    z=q23_mul(z,w)+(*p);
		p++;
	}while (p<k+5);
	z=Q23_PDIV(z,x) ;
	long logx_1;
	if (x<Q_23*32)
		logx_1=q26in_q29out_log(x<<3)-Q_29;
	else
		logx_1=q26in_q29out_log(x)-Q_29+1116391677;//log(8)(Q_29)
	z=z- ((logx_1+64)>>7)-(Q_29>>7)+q29_mul(x , logx_1) + 7708615-zurasi;//0.5 * LOG_2PI

	return z;
}


//ガンマ関数 　 (-0.98<=x<=-0.02,0.02<=x<=6.4)
long q23_tgamma(long x) 
{
	if (x<0)return q23_div(Q23_PI,q23_mul(q23_sin(q23_mul(Q23_PI,x)),q23_exp(q23_lgamma(Q_23-x))));
	return q23_exp(q23_lgamma(x));
}


//ガンマ関数の自然対数 　 (0<x<=17.4)
long q26_lgamma(long x)  
{
	long  w ,z,zurasi=0 ;
	if (x < Q_26*3){
		long v=x;
		x=x+Q_26;
		while (x < Q_26*3) {
			v=q26_mul(v,x);
			x=x+Q_26;
		}
		zurasi=q26_log(v);
	}
	long  x1=Q26_PDIV(Q_26,x);
 	static long k[5]={
 		56489,  	// ( 5.0 / 66.0) / (10 *  9);
 		-39946,  	// (-1.0 / 30.0) / ( 8 *  7);
 		53261 ,  	// ( 1.0 / 42.0) / ( 6 *  5);
 		-186414, 	// (-1.0 / 30.0) / ( 4 *  3);
 		5592405 	// ( 1.0 / 6.0)  / ( 2 *  1);
	};
	long *p=k;
	w=q26_mul(x1,x1);
	z=-128683;	// (-691.0 / 2730.0) / (12 * 11);
	do{
	    z=q26_mul(z,w)+(*p);
		p++;
	}while (p<k+5);
	z=Q26_PDIV(z,x) ;
	long logx=q26in_q29out_log(x);
	z=z- ((logx+4)>>4)+q29_mul(x , logx-Q_29) + 61668921-zurasi;//0.5 * LOG_2PI
	return z;
}


//ガンマ関数 　 (-0.96<=x<=-0.04,0.04<=x<=5.1)
long q26_tgamma(long x) 
{
	if (x<0)return q26_div(Q26_PI,q26_mul(q26_sin(q26_mul(Q26_PI,x)),q26_exp(q26_lgamma(Q_26-x))));
	return q26_exp(q26_lgamma(x));
}


//誤差関数 erf(x) (0<=x<=31) 
long q26sub_erf(long absx)
{
	if (absx<Q_26*1.15){
		//Taylor展開
		// k[10]={-2/918086400,2/76204800,-2/6894720,2/685440,-2/75600,2/9360,- 2/1320,2/216,-2/42,2/10,- 2/3,2};
 		static long k[10]={-156,1567 ,-14203 , 114716 ,-813441 , 4971027 ,-25565282 , 107374182 ,-357913941 , 1073741824 };
		long *p=k;
		absx=absx<<3;
		long xx=q29_mul(absx,absx);
		long y=((-xx+Q_29/2)>>29)+14;//y = -Q_29*2/918086400*xx+Q_29*2/76204800;
		do{
	    	y=q29_mul(y,xx)+(*p);
			p++;
		}while (p<k+10);
		y = q29_mul(y,absx);
		y = q29_mul(y,302896976);//y/sqrt(Q29_PI);
		return (y+4)>>3;	
	}else{
		//Hastingsの近似式
		static long s[5]={50373428,-12274168,27439487,16384718,0};//地球シミュレータセンター　河野明男先生が求めた係数 
		static long t[5]={-97519382,95389461,-19092253,17101324,0};//Hastingsの近似式 
		long u,v,z,w,*p=s,*q=t;
		u = Q26_PDIV(143447719 , 143447719 + absx);
		v = Q26_PDIV(204855578 , 204855578 + absx);
		z = -14814608;
		w = 71229713;
		do{
	    	z=q26_mul(z,u)+(*p);
	    	w=q26_mul(w,v)+(*q);
			p++;
			q++;
		}while (p<s+5);
		z=((17*z+15*w)+16)>>5;//誤差の打消し 

		long xx=q26_mul(absx,absx);
		return Q_26-q26_mul(z,q26_exp(-xx));		
	}
}


//誤差関数 erf(x) (-255<=x<=255)
long q23_erf(long x)
{
	long absx,y;
	if (x>=0) absx=x;else absx=-x;
	if (absx<Q_23*4)
		y=(q26sub_erf(absx<<3)+4)>>3;
	else
		y=Q_23;
	if(x<0) y=-y;
	return y;
}


//誤差関数 erf(x) (-31<=x<=31)
long q26_erf(long x)
{
	long absx,y;
	if (x>=0) absx=x;else absx=-x;
	if (absx<Q_26*4.1)
		y=q26sub_erf(absx);
	else
		y=Q_26;
	if(x<0) y=-y;
	return y;
}


//余誤差関数 1-erf(x) (-255<=x<=255)
long q23_erfc(long x){
	return Q_23-q23_erf(x);
}


//余誤差関数 1-erf(x) (-31<=x<=31)
long q26_erfc(long x){
	return Q_26-q26_erf(x);
}

#undef   QU_MUL
#undef   Q30U_MUL
#undef   Q31U_MUL
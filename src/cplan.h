#ifndef CPLAN_H_
#define CPLAN_H_

//功能：生成余弦曲线，振幅a和角频率w及运行时间t由用户自定义//
//
//   ##参数定义##
//
//   w_:角频率，由用户输入，构造函数初始化
//   a_:振幅，由用户输入，构造函数初始化
//   t_：运行时间，由用户输入，构造函数初始化

class cosCurve
{
private:
	double a_;
	double w_;
    double p_;

public:
    auto getCurve(int count)->double;  //计算当前的位置
    cosCurve(double a, double w, double p)
    {
        a_= a;
        w_=w;
        p_=p;
    }  //构造函数
    ~cosCurve() {} //析构函数
};

#endif




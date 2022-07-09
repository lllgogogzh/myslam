### Bug 小记

#### 1、xxxx未定义的引用

**原因：**一般情况下考虑为：.h文件中有函数名，但是没有函数实现。

**解决办法：**若未自己写的库，要给出函数实现；如果是调用其他库，要链接libraries。

例如：

```c++
//在.h文件中：
class Test
{
public:
	Test();//构造函数，但是没实现
};


//在.cpp文件中，必须实现
Test::Test()
{
    cout<<"construction function called"<<endl;
}
```


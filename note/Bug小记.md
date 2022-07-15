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

#### 2、invalid new-expression of abstract class type ‘class type’

**原因：**出现这个错误原因是new 了一个抽象类出错，说明父类（接口）中有纯虚函数没有实现。接口里的纯虚函数全部需要实现，这样才能new 子类。

**解决办法：**子类需要实现虚函数。

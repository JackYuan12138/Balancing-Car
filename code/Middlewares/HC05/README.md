## 使用方式
1. 引入`my_usart.h`和`stdio.h`
2. 使用`HC05_Init()`初始化
3. 使用`printf()`发送数据(已经重置该库函数)
4. 接收的数据存储在BT_Data结构体
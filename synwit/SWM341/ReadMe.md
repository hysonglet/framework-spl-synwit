1����ʼ��ʱ����Գ�ʼ���ṹ������г�Ա��ȷ��ֵ�����磺

GPIO_initStruct.func = 0;					//���Ź���ΪGPIO
GPIO_initStruct.dir = 1;					//���
GPIO_initStruct.pull_up = 0;
GPIO_initStruct.pull_down = 0;
GPIO_initStruct.open_drain = 0;
GPIO_Init(GPIOA,PIN_2,&GPIO_initStruct);	//GPIOA.2��ʼ��Ϊ������ţ������������������ǿ�©

����Ĵ����ʼ��GPIOA.2Ϊ������ţ��û�ʵ�ʲ��������ù��ڡ���������������©���������Ϣ�������ڵ���GPIO_Init�趨����ģʽǰ�����
GPIO_initStruct������pull_up��pull_down��open_drain������Ա������ȷ��ֵ��������GPIO_initStructΪ�ֲ���������������޷���֤����
����Ա��ȡֵ�������һ��ʱopen_drainȡֵΪ��1������ô��������ž��޷�����ߵ�ƽ�ˡ�����


��������������SPI��UART��TIMR�ȵĳ�ʼ��Ҳ��ͬ����Ҫ�󣬱�������еĳ�ʼ���ṹ���Ա������ȷ��ֵ����


��������Ӧ�������ȷ����Գ�ʼ���ṹ���Ա������ȷ��ȡֵ�����Բο��������Դ������̴��롣��

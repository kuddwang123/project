time   timestamp   #!< 时间戳, 单位us
uint32 code_val    #!< 错误码, enum值

uint8  status        #!< 错误程度, enum值
uint8 FAILED  = 1    # 致命错误
uint8 ERROR   = 2    # 一般错误
uint8 WARNING = 3    # 警告
uint8 NORMAL  = 4    # 正常
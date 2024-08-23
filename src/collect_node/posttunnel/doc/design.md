### postTunnel
 该模块功能向指定服务器上传业务设备文件，目前支持s3存储，涉及的文件包括地图数据，轨迹数据，清扫日志等。

#### 接口
rostopic:"/uploadfile/s3"
ros msg:
String : "filename (with absolute path)"

#### 基本时序
```plantuml
@startuml
skin rose
actor application as app
participant postTunnel as post
boundary iot as iot
queue libcurl as curl
database s3 as s3
app -> post : file upload
post -> iot : get url
iot --> post :return url
post -> curl :http
curl -> s3 :  PUT
@enduml
```

#### 协议
```json
//请求URL
{
  "uploadFileUrl":{
    "type":0,
    "uuid":"1234",
    "file":"map1-1716946233.json"
  }
}

//URL返回
{
  "uploadFileUrl":{
    "type":0,
    "uuid":"1234",
    "url":"xxxx",
    "urlExpire": 120
  }
}

//上传完成上报
{
  "uploadResultReport":{
    "type":0,
    "file":"map1-1716946233.json"
  }
}
```
#### 流程
##### 主业务流程
```plantuml
@startuml
skin rose
start
:load fail msgs from local;
:spin;
fork
: file upload chatter;
note right
  filepath 
end note
if (file exist) then  (yes)

 if (origin file exist in fail queue) then (yes)
   :delete cached file;
   :update msg;
   note right
     update uuid
     update ts
     update new cached file name,
     update md5
   end note
   : replace with new msg;  
 else (no)
   : generate **[file msg]**;
   note right
   {
    "id" : "uuid",
    "originfile": "origin file path",
    "md5": "file md5",
    "cachedfile":"cached file path",
    "expire" :,
    "url" : "",
    "urlgents":
    }
   end note
   : push to fail queue;
 endif

: copy origin file to cache;

 if (origin file md5 == md5 in msg) then(yes)
  
  : pub get s3 url;
 
 else (no)
  : delete cached file;
  : remove the fail msg;
 endif
 : save fail queue msgs locally;
else (no)
endif

fork again
:url msg income;
 note right
   uuid, expire, url
 end note
 if (uuid exist in fail queue) then(yes)
  :update msg;
 
  note right
 fill "url", "expire", "urlgen"
 end note
 :move from fail queue 
 to active queue;
 #pink:**notify**;
 end if
end fork
end

@enduml
```

```plantuml
@startuml
skin rose

start
split

:condition wake;
if (active queue empty) then (no)
 :active queue msg pop front;
    if (origin file md5 == md5 in msg) then (yes)
      if (url expire?) then (yes)
       : pub get s3 url;
       : update file msg;
       : move to fail queue;
      else (no)
      partition Upload Thread {
       :upload cached file;
       if (upload success) then(yes)
         :upload done report;
         :delete msg;
         :delete cached file;
       else (no)
         if (exceed max retry times) then(yes)
           : move to fail queue;    
         else (no)
           : push to active queue;
           #pink:**notify**; 
         endif
      
       endif
      } 
      endif
    else (no)
      :delete msg;
      :delete cached file;
    endif
else (yes)     
endif
split again
:timer trigger;
:load fail msgs from local;
if (origin md5 equal and cached file exist?) then (yes)
  if (url expire or lack?) then (no)
    :move to active queue;
    #pink:**notify**;
  else  (yes)
    :pub get s3 url;
  endif
else (no)
  :remove from fail queue;
  :save fail msgs locally; 
  end
endif

end split
end
@enduml

```
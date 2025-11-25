# 这个是git的相关使用的文件
流程
注因为虚拟环境太大，git不支持100M以上的文件推送，克隆本项目时需要自己在添加虚拟环境
## 初始化
```bash
    git init
```
## 把虚拟环境添加到.gititnore
```bash
    echo "Camera/" >> .gitignore
```
## 创建新分支
```bash
    git branch -M li
```
## 将文件放入缓冲区并提交文件
```bash
    git add .
    git commit -m "robot"
    # 第一次提交需要配置你的用户名和邮箱
```
## 添加远程仓库
```bash
    git remote add myself url
```
## 多人协作要先拉取在提交，
```bash
    git pull url li
    # 第一次的时候需要加u
    git push -u url li
```    
## 这里由于本人刚刚开始把虚拟环境添加进去了，使用这个可以去除在.git里面添加的文件，并且重新提交
```bash
    git rm -r --cached Camera/
    git commit -m "movevirtualdir"
    git push -u myself liquanyan --force

```
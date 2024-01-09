# cartographer原理
## 一、cartographer依赖库
1) eigen : 3.2.9
2) ceres : 1.13.0
3) protobuf : 3.0.0以上，选择3.4.0或3.4.1
4) abseil : 


### 二、安装依赖过程
#### 2-1、 安装依赖
  sudo apt-get install -y \
      cmake \
      g++ \
      git \
      google-mock \
      libboost-all-dev \
      libcairo2-dev \
      libeigen3-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      liblua5.2-dev \
      libsuitesparse-dev \
      libwebp-dev \
      ninja-build \
      protobuf-compiler \
      python-sphinx


#### 2-2、 安装protobuf
     1) 安装protobuf需要的依赖
        
     2) 下载3.4.0版本protobuf

     3) 安装过程
        cd protobuf
 
        git submodule update --init --recursive
        （一般卸载完protoc的各种依赖后添加子模块是不会报错的）
        
        ./autogen.sh
        
        ./configure
        
        make
        如果添加子模块失败，执行make check会报错，但不影响后面的步骤，只是后续使用时会出现一些问题
        make check
        
        sudo make install
        
        sudo ldconfig
        
        protoc --version 输出版本号成功
     
      4) cartographer查找protobuf路径是/usr/bin
          ① 执行复制命令
          sudo cp -f /usr/local/bin/protoc /usr/bin/
          ② 查看是否复制成功
          find /usr/bin -name 'protoc'
          该命令会显示要查找的文件的所在路径


### 三、编译cartographer
    3-1、 创建工作空间
          mkdir  -p carto_ws/src
          cd carto_ws
          wstool init src
    3-2、 克隆代码


    3-3、编译代码
          catkin_make_isolated --install --use-ninja






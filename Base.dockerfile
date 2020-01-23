FROM ubuntu:latest

RUN apt update
RUN apt -y install libuv1-dev libssl-dev libz-dev wget build-essential git

RUN wget -qO- "https://cmake.org/files/v3.15/cmake-3.15.4-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local
RUN apt-get install

RUN git clone https://github.com/uWebSockets/uWebSockets /usr/local/uWebSockets
WORKDIR /usr/local/uWebSockets
RUN git checkout e94b6e1
WORKDIR /usr/local/uWebSockets/build
RUN cmake ..
RUN make 
RUN make install
WORKDIR /usr/local
RUN ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
RUN rm -r uWebSockets

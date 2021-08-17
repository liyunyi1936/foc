import socket    #引入套接字
import threading    #引入并行
udp_data = None
def udp_send(udp_socket):

    while True:
        num1 = '192.168.4.1'
        num2 = 2333
        send_data = input('请输入要发送的数据：')
        send_data = send_data.encode('utf-8')
        udp_socket.sendto(send_data,(num1,num2))  #sendto（发送数据，发送地址）

def udp_recv(udp_socket):
    global udp_data
    while True:
        recv_data = udp_socket.recv(1024)
        recv_data = recv_data.decode('utf-8')
        udp_data = recv_data
        print('收到信息为:%s'%recv_data)

def main():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   #创建套接字
    ip = '192.168.4.2'                                           #服务器ip和端口
    port = 2333
    udp_socket.bind(("192.168.4.2",2333))                                   #服务器绑定ip和端口
    #发送数据
    t=threading.Thread(target=udp_send,args=(udp_socket,))     # Thread函数用于并行
    #接收数据
    t1=threading.Thread(target=udp_recv,args=(udp_socket,))
    t.start()                                                 #并行开始
    t1.start()

if __name__ == '__main__':
    main()
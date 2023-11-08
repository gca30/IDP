#include <SFML/Network/Socket.hpp>
#include <SFML/System/Time.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <iostream>

using namespace std;
using namespace sf;

string ip = "192.168.4.1";
int port = 80;

int main() {
    Texture layout, car; 
    if(!layout.loadFromFile("assets/layout.png")
    || !car.loadFromFile("assets/car.png")) {
        cerr << "Failed to load textures" << endl;
        return -1;
    }
    Sprite carsp(car);
    carsp.setPosition((float)layout.getSize().x / 2 - (float)car.getSize().x / 2 - 5, layout.getSize().y - car.getSize().y - 80);
    RenderWindow window(VideoMode(layout.getSize().x, layout.getSize().y), "IDP WASD Controller");

    // cout << "Enter Arduino IP: ";
    // cin >> ip;
    // TcpSocket socket;
    // if(socket.connect(ip, 5000) != Socket::Done) {
    //     cerr << "Failed to connect to server" << endl;
    //     return -1;
    // }
    Clock clock;

    Event event;
    while (window.isOpen()) {
        while (window.pollEvent(event)) {
            if (event.type == Event::Closed)
                window.close();
        }
        
        if(clock.getElapsedTime() > milliseconds(5)) {
            char data = 0;
            if(Keyboard::isKeyPressed(Keyboard::W))
                data |= 1 << 0;
            if(Keyboard::isKeyPressed(Keyboard::A))
                data |= 1 << 1;
            if(Keyboard::isKeyPressed(Keyboard::S))
                data |= 1 << 2;
            if(Keyboard::isKeyPressed(Keyboard::D))
                data |= 1 << 3;
            // socket.send(&data, 1);
            clock.restart();
        }

        window.clear();
        window.draw(Sprite(layout));
        window.draw(carsp);
        window.display();
    }
    //socket.disconnect();
    return 0;
}

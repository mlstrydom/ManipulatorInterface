#include <Channel.h>
#include <Input.h>
#include <string>


int main()
{
    SerialChannel serial;
    MainSelection mainSelection;

    serial.connect("COM5"); //Set COM port for Arduino

    int menu = getch();
    while(true){
        while(menu == 100 || menu == 109 || menu == 97 ){
            mainSelection.mainKeySelection(menu, serial);
        }
        std::cout << "Wrong Selection - Press d, m or a" << std::endl;
        menu = getch();
    }
    return menu;

}


#include <iostream>
#include "rvc_controller.h"

class TestScanner {
public:
    void check_status() const {
        const int* ptr = nullptr;

        if (ptr == nullptr) {
            constexpr int local_value = 10;
            std::cout << "Scanning..." << local_value << std::endl;
        }
    }
};

int main() {
    TestScanner scanner;
    scanner.check_status();
    RvcController c;
    std::cout << "RVC Controller: " << c.next_action(false,false,false,false) << "\n";
    return 0;
}
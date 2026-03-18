#include <iostream>
#include "rvc_controller.h"

class TestScanner {
public:
    void Check_Status() { // snake_case + non-const -> Clang-Tidy가 싫어함
        // 3. modernize-use-nullptr: NULL 대신 nullptr 사용 권장
        int* ptr = NULL;

        // 4. readability-identifier-naming: VariableCase (camelCase여야 함)
        int My_Local_Variable = 10;

        if (ptr == NULL) {
            std::cout << "Scanning..." << My_Local_Variable << std::endl;
        }
    }
};

int main() {
    TestScanner scanner;
    scanner.Check_Status();
    RvcController c;
    std::cout << "RVC Controller: " << c.next_action(false,false,false,false) << "\n";
    return 0;
}
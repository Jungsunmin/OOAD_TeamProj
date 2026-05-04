#include <iostream>
#include <thread>
#include <chrono>
#include "rvc_controller.h"

class TestScanner {
public:
    void Check_Status() { // snake_case + non-const -> Clang-Tidy�� �Ⱦ���
        // 3. modernize-use-nullptr: NULL ��� nullptr ��� ����
        int* ptr = NULL;

        // 4. readability-identifier-naming: VariableCase (camelCase���� ��)
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
	int unused = 1; // cppcheck: unusedVariable - 'unused' is assigned a value that is never used.
    return 0;
}

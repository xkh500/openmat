// Check compiling and linking something using Qt libraries.
// 32 / 64 bit mismatches can happen.

#include <QWidget>

int main(int, char **)
{
    QWidget w;
    return 0;
}

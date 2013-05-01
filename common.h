
#define LEVEL 1

#define _print(var) std::cout<<#var<<" = "<<(var)<<" ("<<&(var)<<") ("<<__FUNCTION__<<")"<<std::endl
#define _print2(var1, var2) std::cout<<#var1<<" = ["<<(var1)<<","<<(var2)<<") ("<<&(var1)<<"] ("<<__FUNCTION__<<")"<<std::endl
#define _print3(var1, var2, var3) std::cout<<#var1<<" = ["<<(var1)<<","<<(var2)<<","<<(var3)<<"] ("<<&(var1)<<") ("<<__FUNCTION__<<")"<<std::endl
#define _print4(var1, var2, var3, var4) std::cout<<#var1<<" = ["<<(var1)<<","<<(var2)<<","<<(var3)<<","<<(var4)<<"] ("<<&(var1)<<") ("<<__FUNCTION__<<")"<<std::endl
#define _print_name(var) std::cout<<#var<<" ("<<&(var)<<") ("<<__FUNCTION__<<")"
#define _print_name_ln(var) std::cout<<#var<<" ("<<&(var)<<") ("<<__FUNCTION__<<")"<<std::endl
#define ERROR_PRINT(var) std::cout<<"error is "<<#var<<" = "<<(var)<<" ("<<&(var)<<") ("<<__FUNCTION__<<")"<<std::endl

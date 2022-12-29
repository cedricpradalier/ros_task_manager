
#include <task_manager_msgs/msg/encapsulated_message.hpp>
#include <task_manager_msgs/encapsulate_message.h>
#include <ostream>

std::ostream& operator<<(std::ostream& s, const task_manager_msgs::msg::EncapsulatedMessage & v) {
    s << "Type: " << v.type << " md5sum '" << v.md5sum << "' - ";
    return s;
}

int main(int argc, char *argv[]) {
    // ros::init(argc,argv, "test_encaps");

    task_manager_msgs::msg::EncapsulatedMessage E1, E2, E3;
    E1.type = "test/test";
    E1.md5sum = "0123456";
    E1.data.resize(10);
    for (size_t i=0;i<10;i++) {
        E1.data[i] = i;
    }

    printf("Original data\n");
    std::cout << E1 << std::endl;
    for (size_t i=0;i<E1.data.size();i++) {
        printf("%02X ",E1.data[i]);
    }
    printf("\n");

    task_manager_msgs::encapsulate(E2, E1);
    printf("Encapsulated\n");
    std::cout << E2 << std::endl;
    for (size_t i=0;i<E2.data.size();i++) {
        printf("%02X ",E2.data[i]);
    }
    printf("\n");

    task_manager_msgs::decapsulate(E3, E2);
    printf("Recovered data\n");
    std::cout << E3 << std::endl;
    for (size_t i=0;i<E3.data.size();i++) {
        printf("%02X ",E3.data[i]);
    }
    printf("\n");

    // This is not working at this stage (E3.data is not reinitialized
    // correctly by the deserializer)

    return 0;
}



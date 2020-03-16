#ifndef TASK_PARAMETER_LIST_H
#define TASK_PARAMETER_LIST_H

// We need to resort to a macro to handle the way enum are managed by 
// dynamic reconfigure
#define TASK_PARAMETER_ENUM(TASK_CLASS) \
    typedef enum {Clear = TASK_CLASS##_Clear,\
    Push = TASK_CLASS##_Push, Execute = TASK_CLASS##_Execute} ParameterListEnum

namespace task_manager_lib {

    // Generic class to implement the functions required to create classes 
    // designed to be called iteratively to push list of parameters before 
    // executing them. Typically used for trajectory definition.
    // This needs to be inherited from and the push_config function redefined.
    template <class Config, class Storage>
        class ParameterList {
            protected:
                typedef std::list<Storage> StorageList;
                StorageList storage;
            public:
                ParameterList() {}
                virtual ~ParameterList() {}

                
                virtual void push_config(const Config & cfg) = 0; 

                void push_storage(const Storage & s) {storage.push_back(s);}
                void clear() {storage.clear();}
                bool empty() const {return storage.empty();}
                const Storage & front() const {return storage.front();}
                void pop_front() {storage.pop_front();}
                size_t size() {return storage.size();}

                typename StorageList::const_iterator begin() const {
                    return storage.begin();
                }

                typename StorageList::const_iterator end() const {
                    return storage.end();
                }

        };

};

#endif // TASK_PARAMETER_LIST_H

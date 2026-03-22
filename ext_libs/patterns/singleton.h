#ifndef PATTERN_SINGLETON_H_
#define PATTERN_SINGLETON_H_

template <typename T>
class UniqueInstance
{
public:
    UniqueInstance(const UniqueInstance&) = delete;
    UniqueInstance& operator=(const UniqueInstance&) = delete;

    static T* getInstance()
    {
        static T instance;
        return &instance;
    }

private:
    UniqueInstance() {}
    virtual ~UniqueInstance() {}
};

#endif // PATTERN_SINGLETON_H_
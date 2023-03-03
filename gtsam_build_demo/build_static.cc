#include "gtsam_build.h"

std::unique_ptr<ABC> new_abc()
{
    return std::unique_ptr<ABC>(new ABC());
}

void ABC::print() const
{
    cout << "ABC::print()" << endl;
}
void ABC::add(rust::Str str)
{
    m_strs.push_back(str);
}
void ABC::clear()
{
    m_strs.clear();
}
rust::Str ABC::get(int index) const
{
    if (index < m_strs.size())
    {
        return m_strs[index];
    }
    return "";
}

int example()
{
    return 0;
}
//
// Created by eranhe on 6/6/21.
//

#ifndef PE_PRINTFUNC_HPP
#define PE_PRINTFUNC_HPP

template<typename Value, typename... Values>
std::string to_csv2 ( Value v, Values... vs )
{
    std::ostringstream oss;
    using expander = int[];
    oss << v; // first
    (void) expander{ 0, (oss << ";" << vs, void(), 0)... };
    return oss.str();
}

template <typename Arg, typename... Args>
void doPrint(std::ostream& out, Arg&& arg, Args&&... args)
{
    out << std::forward<Arg>(arg);
    ((out << ',' << std::forward<Args>(args)), ...);
}
#endif //PE_PRINTFUNC_HPP

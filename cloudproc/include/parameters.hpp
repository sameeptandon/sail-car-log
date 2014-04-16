template <typename T>
void pyListToVector(const boost::python::object& py_list, std::vector<T>& vec)
{
    int len = boost::python::len(py_list);
    for (int i = 0; i < len; ++i)
    {
        vec.push_back(boost::python::extract<T>(py_list[i]));
    }
}

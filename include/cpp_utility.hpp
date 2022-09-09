#ifndef CPP_UTILITY_HPP
#define CPP_UTILITY_HPP

#include <vector>
#include <limits>
#include <iostream>

template<typename T>
void print_vector(const std::vector<T> &__vec) {
	if (!__vec.size())
		std::cerr<<"print_vector() : Empty vector\n";
	std::cout<<"Printing vector :\n";
	for (int i=0; i<__vec.size(); i++) {
		std::cout<<"i : "<<i<<" :\n"<<__vec[i]<<"\n";
	}
}

template <typename T>
void compare_vectors(const std::vector<T> &__vec0, const std::vector<T> &__vec1) {
	if (!__vec0.size() || !__vec1.size())
		std::cerr<<"compare_vector() : Empty vector\n";
	if (__vec0.size() != __vec1.size())
		std::cerr<<"compare_vector() : Different size of vectors\n";
	std::cout<<"Comparing vector :\n";
	for (int i=0; i<__vec0.size(); i++) {
		std::cout<<"i : "<<i<<" : "<<__vec0[i].isApprox(__vec1[i])<<"\n";
	}
}

#endif

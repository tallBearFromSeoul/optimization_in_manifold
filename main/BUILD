cc_library(
	name = "cpp_utility",
	hdrs = ["include/" + x for x in [
						"cpp_utility.hpp",
	]],
	includes = ["include",
	],
	visibility = ["//main:__pkg__"],
)

cc_library(
	name = "rotation",
	hdrs = ["include/" + x for x in [
						"exp_and_log_map.hpp",
						"jacobians.hpp",
						"slerp.hpp",
						"se3.hpp",
						"conversion.hpp", 
						"utility.hpp",
	]],
	includes = ["include",
	],
	visibility = ["//main:__pkg__"],
)

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")
cmake(
	name = "eigen",
	cache_entries = {
			"CMAKE_C_FLAGS": "-fPIC",
	},
	lib_source = "@eigen//:all_srcs",
	out_headers_only = True,
	includes = ["eigen3/",],
	install = True,
)

cc_binary(
	name = "optimization_in_manifold",
	srcs = ["src/" + x for x in [
						"main.cpp",
	]],
	visibility = ["//visibility:public"],
	deps = [":eigen",
						":cpp_utility",
						":rotation",
	],
)

workspace(name="optimization_in_manifold")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
	name = "rules_foreign_cc",
	sha256 = "6041f1374ff32ba711564374ad8e007aef77f71561a7ce784123b9b4b88614fc",
	strip_prefix = "rules_foreign_cc-0.8.0",
	url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.8.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

# This sets up some common toolchains for building targets. For more details, please see
# https://bazelbuild.github.io/rules_foreign_cc/0.8.0/flatten.html#rules_foreign_cc_dependencies
rules_foreign_cc_dependencies()

_ALL_CONTENT = """\
filegroup(
	name = "all_srcs",
	srcs = glob(["**"]),
	visibility = ["//visibility:public"],
)
"""
http_archive(
  name="eigen",
  build_file_content = _ALL_CONTENT,
  strip_prefix = "eigen-3.4",
  urls=["https://gitlab.com/libeigen/eigen/-/archive/3.4/eigen-3.4.tar.bz2"],
  sha256 = "a6f7aaa7b19c289dfeb33281e1954f19bf2ba1c6cae2c182354d820f535abef8",
)

'''
workspace(name = "EigenDemo")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Eigen
http_archive(
    name = "eigen",
    build_file = "//:eigen.BUILD",
    sha256 = "3a66f9bfce85aff39bc255d5a341f87336ec6f5911e8d816dd4a3fdc500f8acf",
    url = "https://bitbucket.org/eigen/eigen/get/c5e90d9.tar.gz",
    strip_prefix="eigen-eigen-c5e90d9e764e"
)
'''

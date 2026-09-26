load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "analysis",
    srcs = ["analysis.py"],
    deps = [
        "//software/stats/logs:event_log",
    ],
)

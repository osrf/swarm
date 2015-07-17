/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/algorithm/string/replace.hpp>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/compiler/code_generator.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/descriptor.pb.h>

#include "IgnGenerator.hh"

namespace google {
namespace protobuf {
namespace compiler {
namespace cpp {

//////////////////////////////////////////////////
bool IgnGenerator::Generate(const FileDescriptor *_file,
    const std::string &/*_parameter*/,
    OutputDirectory *_generatorContext,
    std::string * /*_error*/) const
{
  std::string headerFilename = _file->name();
  boost::replace_last(headerFilename, ".proto", ".pb.h");

  std::string sourceFilename = _file->name();
  boost::replace_last(sourceFilename, ".proto", ".pb.cc");

  // GCC system_header pragma:
  // treat the rest of this file as a system header
  {
    std::unique_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(headerFilename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#pragma GCC system_header", "name", "includes");
  }

  // Ignore -Wshadow diagnostic
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(sourceFilename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#pragma GCC diagnostic ignored \"-Wshadow\"",
        "name", "includes");
  }

  // Ignore -Wswitch-default diagnostic
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(sourceFilename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#pragma GCC diagnostic ignored \"-Wswitch-default\"",
        "name", "includes");
  }

  // Add boost shared point include
  {
    scoped_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(headerFilename, "includes"));
    io::Printer printer(output.get(), '$');

    printer.Print("#include <boost/shared_ptr.hpp>\n", "name", "includes");
    printer.Print("#include <swarm/Helpers.hh>\n", "name", "includes");
  }

  // Add boost shared typedef
  {
    std::unique_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(headerFilename,
          "namespace_scope"));
    io::Printer printer(output.get(), '$');

    std::string package = _file->package();
    boost::replace_all(package, ".", "::");

    std::string ptrType = "typedef boost::shared_ptr<" + package
      + "::" + _file->message_type(0)->name() + "> "
      + _file->message_type(0)->name() + "Ptr;\n";

    printer.Print(ptrType.c_str(), "name", "namespace_scope");
  }

  // Add const boost shared typedef
  {
    std::unique_ptr<io::ZeroCopyOutputStream> output(
        _generatorContext->OpenForInsert(headerFilename,
          "global_scope"));

    io::Printer printer(output.get(), '$');

    std::string package = _file->package();
    boost::replace_all(package, ".", "::");

    std::string constType = "typedef const boost::shared_ptr<"
      + package + "::" + _file->message_type(0)->name()
      + " const> Const" + _file->message_type(0)->name() + "Ptr;";

    printer.Print(constType.c_str(), "name", "global_scope");
  }

  return true;
}
// namespace cpp
}
// namespace compiler
}
// namespace protobuf
}
// namespace google
}

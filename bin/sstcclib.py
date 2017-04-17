import os
import sys
from configlib import getstatusoutput

helpText = """The following environmental variables can be defined for the SST compiler
SSTMAC_VERBOSE=0 or 1:        produce verbose output from the SST compiler (default 0)
SSTMAC_DELETE_TEMPS=0 or 1:   remove all temp source-to-source files (default 1)
SSTMAC_SRC2SRC=0 or 1: run a source-to-source pass converting globals to TLS (default 1)
"""

sstmac_libs = [
'-lsprockit',
'-lundumpi',
'-lsstmac',
]


from sstccvars import sstmac_default_ldflags, sstmac_extra_ldflags, sstmac_cppflags
from sstccvars import prefix, exec_prefix, includedir, cc, cxx, cxxflags, cflags
from sstccvars import includedir
from sstccvars import sst_core
from sstccvars import so_flags
from sstccvars import clang_cppflags, clang_ldflags, clang_libtooling_cxxflags, clang_libtooling_cflags

def cleanFlag(flag):
    return flag.replace("${includedir}", includedir).replace("${exec_prefix}", exec_prefix).replace("${prefix}",prefix)

clangCppArgs = [
  cleanFlag("-I${includedir}/sstmac/clang_replacements"),
]
clangCxxArgs = [
  "-std=c++1y",
  "-stdlib=libc++", 
]
clangCxxArgs.extend(clang_libtooling_cxxflags.strip().split())

def addClangArg(a, ret):
  ret.append("--extra-arg=%s" % a)

def addClangArgs(argList, ret):
  for a in argList:
    addClangArg(a,ret)
  return ret

haveClangSrcToSrc = bool(clang_cppflags)
clangDeglobal = None
if haveClangSrcToSrc:
  clangDeglobal = os.path.join(prefix, "bin", "sstmac_clang_deglobal")


new_cppflags = []
for entry in sstmac_cppflags:
  new_cppflags.append(cleanFlag(entry))
sstmac_cppflags = new_cppflags

sstmac_ldflags = []
for entry in sstmac_default_ldflags:
  sstmac_ldflags.append(cleanFlag(entry))
for entry in sstmac_libs:
  sstmac_ldflags.append(cleanFlag(entry))

sstCppFlagsStr=" ".join(sstmac_cppflags)
ldflagsStr =  " ".join(sstmac_ldflags)
ld = cc 
repldir = os.path.join(includedir, "sstmac", "replacements")
repldir = cleanFlag(repldir)

import sys
def argify(x):
  if ' ' in x: 
    return "'%s'" % x
  else:
    return x

sysargs = sys.argv[1:]


srcFiles = False
asmFiles = False
verbose = False
delTempFiles = True
givenFlags = []
givenCompilerFlags = []
controlArgs = []
linkerArgs = []
sourceFiles = []
objectFiles = []
objTarget = None
getObjTarget = False
for arg in sysargs:
  sarg = arg.strip().strip("'")
  if sarg.endswith('.o'):
    objectFiles.append(sarg)
    objTarget = sarg
    getObjTarget=False
  elif sarg.startswith("-Wl"):
    linkerArgs.append(sarg)
  elif sarg.startswith("-O"):
    givenCompilerFlags.append(sarg)
  elif sarg == "-g":
    givenCompilerFlags.append(sarg)
  elif sarg.endswith('.cpp') or sarg.endswith('.cc') or sarg.endswith('.c'):
    srcFiles = True
    sourceFiles.append(sarg)
  elif sarg.endswith('.S'):
    asmFiles = True
  elif sarg == "--verbose":
    verbose = True
  elif sarg in ("-c","-E"):
    controlArgs.append(sarg)
  elif sarg in ('-o',):
    getObjTarget=True
  elif getObjTarget:
    exeTarget = sarg
    objTarget = sarg
    getObjTarget=False
  else:
    givenFlags.append(sarg)
if sst_core:
  givenFlags.append(" -DSSTMAC_EXTERNAL_SKELETON")



if sourceFiles and len(objectFiles) > 1:
  sys.exit("Specified multiple object files for source compilation: %" % " ".join(objectFiles))

if "SSTMAC_VERBOSE" in os.environ:
  flag = int(os.environ["SSTMAC_VERBOSE"])
  verbose = verbose or flag
if "SSTMAC_DELETE_TEMPS" in os.environ:
  flag = int(os.environ["SSTMAC_DELETE_TEMPS"])
  delTempFiles = delTempFiles and flag
  
def runCmdArr(cmdArr):
  if cmdArr:
    cmd = " ".join(cmdArr)
    if verbose: sys.stderr.write("%s\n" % cmd)
    return os.system(cmd)
  else:
    return 0

def run(typ, extralibs="", includeMain=True, makeLibrary=False, redefineSymbols=True):
    directIncludes = []
    global ldflagsStr
    global sstCppFlagsStr
    import os
    
    if type == "c++":
      directIncludes.append("-include cstdint")
    else:
      directIncludes.append("-include stdint.h")
    directIncludes.append("-include sstmac/compute.h")

    src2src = True
    if "SSTMAC_SRC2SRC" in os.environ:
      src2src = int(os.environ["SSTMAC_SRC2SRC"])

    if sys.argv[1] == "--version" or sys.argv[1] == "-V":
      import inspect, os
      pathStr = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
      print(pathStr)
      cmd = "%s %s" % (cxx, sys.argv[1])
      os.system(cmd)
      sys.exit()
    elif sys.argv[1] == "--flags":
      sys.stderr.write("LDFLAGS=%s\n" % ldflagsStr)
      sys.stderr.write("CPPFLAGS=%s\n" % sstCppFlagsStr)
      sys.stderr.write("CXXFLAGS=%s\n" % cxxflags)
      sys.exit()
    elif sys.argv[1] == "--help":
      cmd = "%s --help" % (cxx)
      os.system(cmd)
      sys.stderr.write(helpText)
      sys.exit()

    compilerFlagsStr = ""
    compiler = ""
    cxxCmd = ""
    if includeMain:
      extralibs += " -lsstmac_main"
    #always c++ no matter what for now
    if typ.lower() == "c++":
      compilerFlagsStr = cleanFlag(cxxflags)
      compiler = cxx
      ld = cxx
    elif typ.lower() == "c":
      compilerFlagsStr = cleanFlag(cflags)
      compiler = cc
      ld = cxx #always use c++ for linking since we are bringing a bunch of sstmac C++ into the game
    cxxFlagsStr = cleanFlag(cxxflags)
    cFlagsStr = cleanFlag(cflags)
    objectFilesStr = " ".join(objectFiles)

    compilerFlagsArr = compilerFlagsStr.split()
    compilerFlags = []
    for entry in compilerFlagsArr:
      if entry[:2] == "-O": #do not send optimization flags forward
        pass
      elif entry == "-g": #do not send debug flags forward
        pass
      else:
        compilerFlags.append(entry)
    compilerFlagsStr = " ".join(compilerFlags)

    directIncludesStr = " ".join(directIncludes)

    extraCppFlags = []
    if redefineSymbols:
        extraCppFlags = [
        "-I%s/include/sumi" % prefix,
        "-DSSTMAC=1",
        "-D__thread=thread_local_not_yet_allowed",
        "-Dthread_local=thread_local_not_yet_allowed",
      ]

    if asmFiles:
        extraCppFlags = [] #add nothing

    if "--no-integrated-cpp" in sysargs:
        extraCppFlags = [] #add nothing

    ldpathMaker = "-Wl,-rpath,%s/lib" % prefix

    if redefineSymbols: 
      extraCppFlags.insert(0,"-I%s" % repldir)

    
    cxxCmdArr = []
    ldCmdArr = []
    arCmdArr = []
    ppCmdArr = []
    ppOnly = "-E" in controlArgs
    runClang = haveClangSrcToSrc and src2src 
    controlArgStr = " ".join(controlArgs)
    extraCppFlagsStr = " ".join(extraCppFlags)
    givenFlagsStr = " ".join(givenFlags)
    givenCompilerFlagsStr = " ".join(givenCompilerFlags)
    srcFileStr = " ".join(sourceFiles)
    if '-c' in sysargs or ppOnly:
      runClang = runClang and (not ppOnly)
      if runClang:
        ppCmdArr = [
          compiler, 
          "-include sstmac/skeleton.h",
          directIncludesStr,
          extraCppFlagsStr, 
          givenFlagsStr,
          givenCompilerFlagsStr,
          sstCppFlagsStr,
          compilerFlagsStr, 
          "-E"
        ]
        #we run clang on a direct source file with no includes
        #only put cxxflags in the cmd arr for now
        cxxCmdArr = [
          compiler,
          compilerFlagsStr,
          givenCompilerFlagsStr
        ]
      else: 
        cxxCmdArr = [
          compiler, 
          extraCppFlagsStr, 
          givenFlagsStr,
          givenCompilerFlagsStr,
          sstCppFlagsStr, 
          compilerFlagsStr, 
          controlArgStr,
          srcFileStr
        ]
    elif objTarget and srcFiles:
      runClang = True
      cxxCmdArr = [
        compiler, 
        extraCppFlagsStr, 
        sstCppFlagsStr, 
        givenFlagsStr,
        compilerFlagsStr, 
        givenCompilerFlagsStr,
        args, 
        ldflagsStr, 
        compilerFlagsStr,
        extralibs, 
        ldpathMaker
      ]
    elif objTarget:
      global verbose
      global exeTarget
      runClang = False

      if "fPIC" in cxxflags or "fPIC" in sstCppFlagsStr:
        arCmdArr = [
          ld,
          so_flags,
          objectFilesStr,
          ldflagsStr,
          givenFlagsStr,
          compilerFlagsStr,
          ldpathMaker,
          "-o",
          "lib" + exeTarget + ".so",
        ]
        arCmdArr.extend(linkerArgs)

      if not sst_core:
        ldCmdArr = [
          ld,
          objectFilesStr,
          extralibs,
          ldflagsStr, 
          givenFlagsStr,
          compilerFlagsStr,
          extralibs, 
          ldpathMaker,
          "-o",
          exeTarget
        ]
        ldCmdArr.extend(linkerArgs)
    else: #all in one
      cxxCmdArr = [
        compiler, 
        extraCppFlagsStr, 
        sstCppFlagsStr, 
        compilerFlagsStr, 
        givenCompilerFlagsStr,
        ldflagsStr, 
        extralibs, 
        ldpathMaker
      ]

    clangExtraArgs = []
    if runClang:
      #this is more complicated - we have to use clang to do a source to source transformation
      #then we need to run the compiler on that modified source
      for srcFile in sourceFiles:
        ppTmpFile = "pp." + srcFile
        cmdArr = ppCmdArr[:]
        cmdArr.append(srcFile)
        cmdArr.append("> %s" % ppTmpFile)
        ppCmd = " ".join(cmdArr) 
        if verbose: sys.stderr.write("%s\n" % ppCmd)
        rc = os.system(ppCmd)
        if not rc == 0:
          return rc

        srcRepl = "sst.pp." + srcFile
        cxxInitSrcFile = "sstGlobals.pp." + srcFile + ".cpp"

        clangCmdArr = [clangDeglobal]
        if typ == "c++":
          addClangArgs(clangCxxArgs, clangCmdArr)
          addClangArgs(clang_libtooling_cxxflags.split(), clangCmdArr)
        else:
          addClangArg(clang_libtooling_cflags, clangCmdArr)
        clangCmdArr.append(ppTmpFile)
        clangCmdArr.append("--")
        clangCmd = " ".join(clangCmdArr)
        if verbose: sys.stderr.write("%s\n" % clangCmd)
        rc = os.system(clangCmd)
        if not rc == 0:
          if delTempFiles:
            os.system("rm -f %s" % ppTmpFile)
            os.system("rm -f %s" % srcRepl)
            os.system("rm -f %s" % cxxInitSrcFile)
          return rc

        #the source to source generates temp .cc files
        #we need the compile command to generate .o files from the temp .cc files
        #update the command to point to them
        cmdArr = cxxCmdArr[:]
        if objTarget:
          objRepl = "sst." + objTarget
          cmdArr.append("-o")
          cmdArr.append(objRepl)
        cmdArr.append("-c")
        cmdArr.append(srcRepl)
        cmdArr.append("--no-integrated-cpp")
        cxxCmd = " ".join(cmdArr)
        if verbose: sys.stderr.write("%s\n" % cxxCmd)
        rc = os.system(cxxCmd)
        if not rc == 0:
          if delTempFiles:
            os.system("rm -f %s" % ppTmpFile)
            os.system("rm -f %s" % objRepl)
            os.system("rm -f %s" % cxxInitSrcFile)
          return rc

        #now we generate the .o file containing the CXX linkage 
        #for global variable CXX init - because C is stupid
        cxxInitObjFile = "sstGlobals." + srcFile + ".o"
        cxxInitCmdArr = [
          cxx,
          cxxFlagsStr,
          sstCppFlagsStr,
          "-o",
          cxxInitObjFile,
          "-I%s/include" % prefix,
          "-c",
          cxxInitSrcFile
        ]
        cxxInitCompileCmd = " ".join(cxxInitCmdArr)
        if verbose: sys.stderr.write("%s\n" % cxxInitCompileCmd)
        rc = os.system(cxxInitCompileCmd)
        if delTempFiles:
          os.system("rm -f %s" % ppTmpFile)
          os.system("rm -f %s" % cxxInitSrcFile)
        if not rc == 0:
          return rc


      #some idiots generate multiple .o files at once
      manyObjects = objTarget == None #no specific target specified
      mergeCmdArr = ["%s -Wl,-r -nostdlib" % compiler]
      for srcFile in sourceFiles:
        srcFileNoSuffix = ".".join(srcFile.split(".")[:-1])
        srcObjTarget = srcFileNoSuffix + ".o"
        srcTformObjFile = "sst." + srcObjTarget
        cxxInitObjFile = "sstGlobals." + srcFile + ".o"
        #now we have to merge the src-to-src generated .o with cxx linkage .o
        if manyObjects:
          #we need to generate a .o for each source file
          cxxMergeCmd = "%s -Wl,-r %s %s -o %s" % (cxx, srcTformObjFile, cxxInitObjFile, srcObjTarget)
          if verbose: sys.stderr.write("%s\n" % cxxMergeCmd)
          rc, output = getstatusoutput(cxxMergeCmd)
          if delTempFiles:
            os.system("rm -f %s %s %s" % (srcTformObjFile, cxxInitObjFile, srcRepl))
          if not rc == 0:
            return rc
        else:
          mergeCmdArr.append("%s %s" % (srcTformObjFile, cxxInitObjFile))
      if not manyObjects:
        mergeCmdArr.append("-o %s" % objTarget)
        mergeCmd = " ".join(mergeCmdArr)
        if verbose: sys.stderr.write("%s\n" % mergeCmd)
        rc, output = getstatusoutput(mergeCmd)
        if delTempFiles:
          os.system("rm -f %s %s %s" % (srcTformObjFile, cxxInitObjFile, srcRepl))
        if not rc == 0:
          sys.stderr.write("deglobal merge error on %s:\n%s\n" % (objTarget, output))
      return rc
          
    else:
      rc = runCmdArr(cxxCmdArr)
      if not rc == 0: return rc
      rc = runCmdArr(ldCmdArr)
      if not rc == 0: return rc
      rc = runCmdArr(arCmdArr)
      return rc





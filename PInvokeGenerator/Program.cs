using System;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using ClangSharp.Interop;
using ClangSharp;

enum ExportSetting
{
    Default,
    Export,
    Ignore
}

unsafe class Bullet4UnityPInvokeGenerator
{
    class ClassInfo
    {
        public string FullyQualifiedName;
        public string MethodPrefix;
        public ClassInfo Base;
        public List<ClassInfo> DerivedClasses = new List<ClassInfo>();
        public CXCursor Handle;
        public List<ClassInfo> ChildClasses = new List<ClassInfo>();
        public List<CXCursor> Methods = new List<CXCursor>();
        public HashSet<CXCursor> Enums = new HashSet<CXCursor>();
        public bool IsManagedStruct;
        public bool IsAligned;
        public ExportSetting Export;

        public ClassInfo()
        {
        }

        public ClassInfo(CXCursor handle)
        {
            Handle = handle;
            FullyQualifiedName = GetFullyQualifiedName(handle);
            MethodPrefix = (FullyQualifiedName + "::").Replace("::", "__");
        }

        public override string ToString()
        {
            return Handle.ToString();
        }
    }

    static readonly Regex _regexFuncName = new Regex("(.+)_([0-9]+)$");

    static readonly string[] _defaultClangCommandLineArgs = new string[]
{
        "-std=c++14",                           // The input files should be compiled for C++ 11
        "-xc++",                                // The input files are C++
        "-Wno-pragma-once-outside-header",      // We are processing files which may be header files
    };

    Dictionary<CXCursor, ClassInfo> _allClasses = new Dictionary<CXCursor, ClassInfo>();
    Dictionary<string, ClassInfo> _classMap = new Dictionary<string, ClassInfo>();

    ClassInfo _root = new ClassInfo();
    HashSet<CXCursor> _exportingEnums = new HashSet<CXCursor>();

    public void Run(string[] args, CodeBuilder result)
    {
        ParseCommandLineArgs(args, out var commandLineArgs, out var sourceContents);

        using (var unsavedFile = CXUnsavedFile.Create("Source", sourceContents))
        using (var index = Index.Create(false, true))
        {
            var unsavedFiles = new CXUnsavedFile[] { unsavedFile };

            var handle = CXTranslationUnit.Parse(
                index.Handle,
                unsavedFile.FilenameString,
                commandLineArgs,
                unsavedFiles,
                CXTranslationUnit_Flags.CXTranslationUnit_IncludeAttributedTypes      // Include attributed types in CXType
                | CXTranslationUnit_Flags.CXTranslationUnit_VisitImplicitAttributes    // Implicit attributes should be visited
                | CXTranslationUnit_Flags.CXTranslationUnit_SkipFunctionBodies
            );

            var translationUnit = TranslationUnit.GetOrCreate(handle);

            CheckTranslationUnitErrors(translationUnit, sourceContents);
            VisitTranslationUnit(translationUnit);
        }

        WriteNamespace(result);

    }

    void ParseCommandLineArgs(string[] args, out string[] outCommandLineArgs, out string outSource)
    {
        string pInvokeSourceDir = args[0];
        string bulletIncludeDir = args[1];

        List<string> sourceLines = new List<string>();

        foreach (var path in Directory.GetFiles(pInvokeSourceDir, "*.cpp"))
        {
            sourceLines.AddRange(File.ReadAllLines(path));
        }

        outSource = string.Join("\n", sourceLines.ToArray());

        List<string> commandLineArgs = new List<string>(_defaultClangCommandLineArgs);
        commandLineArgs.Add("-I"+ bulletIncludeDir);
        commandLineArgs.Add("-I"+ pInvokeSourceDir);
        commandLineArgs.Add("-D_PINVOKEGENERATOR");
        commandLineArgs.Add("-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH");

        for (int i = 2; i < args.Length; ++i)
        {
            commandLineArgs.Add(args[i]);
        }

        outCommandLineArgs = commandLineArgs.ToArray();
    }

    void WriteNamespace(CodeBuilder code)
    {
        code.Append("using UnityEngine;");
        code.Append("using System;");
        code.Append("using System.Runtime.InteropServices;");
        code.Append("using Bullet;");
        code.Append("namespace Bullet");
        code.BeginScope();

        WriteLibraryClass(code);

        code.Append("public interface IUnmanagedObject");
        code.BeginScope();
        code.Append("System.IntPtr Ptr { get; set; }");
        code.EndScope();

        WriteClassEnums(code, _root);

        WriteChildClasses(code, _root);

        code.EndScope();

        WriteExtensions(code);
    }

    void WriteLibraryClass(CodeBuilder code)
    {
        code.Append("internal static partial class Library");
        code.BeginScope();

        code.Append("#if UNITY_EDITOR");
        code.Append("public const string Name = \"bullet4unity_MinSizeRel\";");
        code.Append("#else");
        code.Append("public const string Name = \"__Internal\";");
        code.Append("#endif");

        WritePInvokes(code);
        code.EndScope();
    }

    void WritePInvokes(CodeBuilder code)
    {
        foreach (var classInfo in _allClasses.Values)
        {
            foreach (var method in classInfo.Methods)
            {
                WritePInvoke(code, classInfo, method);
            }
        }
    }

    void WritePInvoke(CodeBuilder code, ClassInfo classInfo, CXCursor methodHandle)
    {
        string retvalType;

        if (methodHandle.ResultType.kind == CXTypeKind.CXType_Void)
        {
            retvalType = "void";
        }
        else if (methodHandle.ResultType.kind == CXTypeKind.CXType_Pointer)
        {
            retvalType = "IntPtr";
        }
        else if (_allClasses.ContainsKey(methodHandle.ResultType.CanonicalType.Declaration))
        {
            retvalType = GetDottedName(methodHandle.ResultType.CanonicalType.Declaration);
        }
        else
        {
            retvalType = GetDottedName(methodHandle.ResultType.CanonicalType.Spelling.ToString());
        }

        string args = "";
        int numArgs = clang.Cursor_getNumArguments(methodHandle);

        for (int i = 0; i < numArgs; ++i)
        {
            var arg = clang.Cursor_getArgument(methodHandle, (uint)i);

            if (args.Length > 0)
            {
                args += ", ";
            }

            string typeName;
            if (arg.Type.kind == CXTypeKind.CXType_Pointer)
            {
                typeName = "IntPtr";
            }
            else if (_allClasses.ContainsKey(arg.Type.CanonicalType.Declaration))
            {
                typeName = GetDottedName(arg.Type.CanonicalType.Declaration);
            }
            else
            {
                typeName = GetCSTypeName(arg.Type.CanonicalType, false);
            }

            args += typeName + " " + arg.Spelling.ToString();
        }

        string marshal = "";
        if (methodHandle.ResultType.kind == CXTypeKind.CXType_Bool)
        {
            marshal = "[return: MarshalAs(UnmanagedType.U1)]";
        }

        code.Append("[DllImport(Library.Name)]" + marshal + " internal extern static {0} {1}({2});", retvalType, methodHandle.Spelling.ToString(), args);
    }

    void WriteExtensions(CodeBuilder code)
    {
        int extIndex = 0;

        code.Append("public static class BulletDefaultExt");
        code.BeginScope();
        code.Append("public static bool isValid<T>(this T self) where T: unmanaged, Bullet.IUnmanagedObject");
        code.BeginScope();
        code.Append("return self.Ptr != System.IntPtr.Zero;");
        code.EndScope();
        code.EndScope();

        foreach (var classInfo in _allClasses.Values)
        {
            if (classInfo.Export == ExportSetting.Export)
            {
                WriteClassExtensions(code, classInfo, ref extIndex);
            }
        }
    }

    void WriteClassExtensions(CodeBuilder code, ClassInfo classInfo, ref int extIndex)
    {
        if (classInfo.IsManagedStruct || classInfo.Methods.Count <= 0)
        {
            return;
        }

        List<CXCursor> methodsToExport = new List<CXCursor>();

        foreach (var method in classInfo.Methods)
        {
            if (!IsMethodStatic(method))
            {
                methodsToExport.Add(method);
            }
        }

        if (methodsToExport.Count > 0)
        {
            code.Append("public static class BulletExt{0}", extIndex.ToString());
            code.BeginScope();

            foreach (var method in methodsToExport)
            {
                WriteMethod(code, classInfo, method, true);
            }

            code.EndScope();
            ++extIndex;
        }
    }

    void WriteChildClasses(CodeBuilder code, ClassInfo classInfo)
    {
        foreach (var childClass in classInfo.ChildClasses)
        {
            if (childClass.Export == ExportSetting.Export)
            {
                WriteClass(code, childClass);
            }
        }
    }

    void WriteClassFields(CodeBuilder code, ClassInfo classInfo)
    {
        classInfo.Handle.VisitChildren((cursor, parent, clientData) =>
        {
            if (cursor.kind == CXCursorKind.CXCursor_FieldDecl)
            {
                string fieldTypeName;

                if (cursor.Type.kind == CXTypeKind.CXType_Pointer)
                {
                    fieldTypeName = GetCSTypeName(cursor.Type.PointeeType, false);
                }
                else
                {
                    fieldTypeName = GetCSTypeName(cursor.Type, false);
                }

                code.Append("public {0} {1};", fieldTypeName, cursor.DisplayName.ToString());
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, clientData: new CXClientData());
    }

    void WriteUpcastOperator(CodeBuilder code, ClassInfo classInfo)
    {
        string className = GetDottedName(classInfo.FullyQualifiedName);

        for (ClassInfo baseRef = classInfo.Base; baseRef != null; baseRef = baseRef.Base)
        {
            string baseName = GetDottedName(baseRef.FullyQualifiedName);

            code.Append("public static implicit operator {0}(in {1} src)", baseName, className);
            code.BeginScope();
            code.Append("return new {0}() {{ Ptr = src.Ptr }};", baseName);
            code.EndScope();
        }
    }

    void WriteClass(CodeBuilder code, ClassInfo classInfo)
    {
        if (classInfo.IsManagedStruct)
        {
            if (classInfo.IsAligned)
            {
                long alignment = clang.Type_getAlignOf(classInfo.Handle.Type);
                long size = clang.Type_getSizeOf(classInfo.Handle.Type);
                size = (size + alignment - 1) / alignment * alignment + alignment;
                code.Append("[StructLayout(LayoutKind.Explicit, Size = {0})]", size.ToString());
            }

            code.Append("public unsafe struct {0}", classInfo.Handle.Spelling.ToString());
        }
        else
        {
            string interfaceName = GetInterfaceName(classInfo.Handle);

            code.Append("public interface {0} : {1} {{}}", interfaceName, classInfo.Base != null ? GetFullInterfaceName(classInfo.Base.Handle) : "IUnmanagedObject");
            code.Append("public unsafe struct {0} : {1}", classInfo.Handle.Spelling.ToString(), interfaceName);
        }

        code.BeginScope();

        if (classInfo.IsManagedStruct)
        {
            if (!classInfo.IsAligned)
            {
                WriteClassFields(code, classInfo);
            }
        }
        else
        {
            code.Append("public IntPtr Ptr { get; set; }");

            WriteUpcastOperator(code, classInfo);
        }

        WriteClassEnums(code, classInfo);
        WriteClassMethods(code, classInfo);

        if (!classInfo.IsManagedStruct)
        {
            WriteChildClasses(code, classInfo);
        }

        code.EndScope();
    }

    void WriteClassMethods(CodeBuilder code, ClassInfo classInfo)
    {
        foreach (var method in classInfo.Methods)
        {
            if (classInfo.IsManagedStruct || IsMethodStatic(method))
            {
                WriteMethod(code, classInfo, method, false);
            }
        }
    }

    static bool IsMethodStatic(CXCursor functionHandle)
    {
        bool result = false;

        functionHandle.VisitChildren((cursor, parent, clientData) =>
        {
            if (cursor.kind == CXCursorKind.CXCursor_AnnotateAttr && cursor.ToString() == "STATIC")
            {
                result = true;
                return CXChildVisitResult.CXChildVisit_Break;
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, clientData: new CXClientData());

        return result;
    }

    string GetCSTypeName(CXType type, bool retval)
    {
        switch (type.kind)
        {
            case CXTypeKind.CXType_Void:
                return "void";
            case CXTypeKind.CXType_UInt:
                return "uint";
            case CXTypeKind.CXType_Bool:
                return "[MarshalAs(UnmanagedType.U1)]bool";
        }
        
        if (type.Declaration.IsInvalid)
        {
            return type.Spelling.ToString();
        }

        return GetDottedName(type.Declaration);
    }

    string GetDefaultParameter(CXCursor handle)
    {
        string result = null;

        handle.VisitChildren((cursor, parent, client_data) =>
        {
            if (cursor.kind == CXCursorKind.CXCursor_CXXBoolLiteralExpr
            || cursor.kind == CXCursorKind.CXCursor_IntegerLiteral
            || cursor.kind == CXCursorKind.CXCursor_FloatingLiteral
            || cursor.kind == CXCursorKind.CXCursor_CompoundLiteralExpr
            || cursor.kind == CXCursorKind.CXCursor_FirstExpr
            || cursor.kind == CXCursorKind.CXCursor_BinaryOperator)
            {
                CXSourceRange range = clang.getCursorExtent(cursor);
                CXToken* tokens = null;
                uint numTokens = 0;
                clang.tokenize(cursor.TranslationUnit, range, &tokens, &numTokens);

                for (int i = 0; i < numTokens; ++i)
                {
                    result += clang.getTokenSpelling(cursor.TranslationUnit, tokens[i]).ToString();
                }

                clang.disposeTokens(cursor.TranslationUnit, tokens, numTokens);
                return CXChildVisitResult.CXChildVisit_Break;
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, new CXClientData());

        return result;
    }

    void WriteMethod(CodeBuilder code, ClassInfo classInfo, CXCursor handle, bool extension)
    {
        bool isStatic = IsMethodStatic(handle);
        bool isExtensionOrStatic = isStatic || extension;

        System.Diagnostics.Debug.Assert(!isStatic || !extension);

        string methodName = GetMethodName(handle, classInfo);

        var retType = clang.getCursorResultType(handle);

        string retTypeName;
        bool returnsPtr = false;
        bool returnsClassPtr = false;

        if (retType.kind != CXTypeKind.CXType_Void)
        {
            if (retType.kind == CXTypeKind.CXType_Pointer)
            {
                returnsPtr = true;

                var pointee = clang.getPointeeType(retType).Declaration;

                if (_allClasses.ContainsKey(pointee))
                {
                    returnsClassPtr = true;
                    retTypeName = GetDottedName(pointee);
                }
                else
                {
                    retTypeName = "IntPtr";
                }
            }
            else if (_allClasses.ContainsKey(retType.CanonicalType.Declaration))
            {
                retTypeName = GetDottedName(retType.CanonicalType.Declaration);
            }
            else
            {
                retTypeName = GetDottedName(retType.CanonicalType.Spelling.ToString());
            }
        }
        else
        {
            retTypeName = "void";
        }

        List<string> constraints = new List<string>();
        List<string> funcArgs = new List<string>();
        List<string> callArgs = new List<string>();
        List<string> fixedArgs = new List<string>();

        int numArgs = clang.Cursor_getNumArguments(handle);

        for (int i = 0; i < numArgs; ++i)
        {
            var arg = clang.Cursor_getArgument(handle, (uint)i);
            var argType = arg.Type;

            string argName = arg.Spelling.ToString();
            string argTypeName = GetCSTypeName(arg.Type, false);
            string defaultParam = GetDefaultParameter(arg);

            if (i == 0 && !isExtensionOrStatic)
            {
                fixedArgs.Add(string.Format("fixed (void* _{0} = &this)", argName));
                callArgs.Add(string.Format("(IntPtr)_{0}", argName));
            }
            else if (argType.kind == CXTypeKind.CXType_Pointer)
            {
                argType = clang.getPointeeType(argType).CanonicalType;
                argTypeName = GetCSTypeName(argType, false);

                _allClasses.TryGetValue(argType.Declaration, out var argClassInfo);

                if (argClassInfo != null && !argClassInfo.IsManagedStruct && argClassInfo.Export != ExportSetting.Ignore)
                {
                    if (defaultParam == "0")
                    {
                        defaultParam = " = default";
                    }

                    if (i == 0 && extension)
                    {
                        funcArgs.Add(string.Format("{2}this T{0} {1}", constraints.Count, argName, GetAnnotations(handle).Contains("DESTRUCTOR") ? "ref " : ""));
                    }
                    else
                    {
                        funcArgs.Add(string.Format("T{0} {1}{2}", constraints.Count, argName, defaultParam));
                    }
                    callArgs.Add(string.Format("{0}.Ptr", argName));
                    constraints.Add(GetFullInterfaceName(argType.Declaration));
                }
                else if (argTypeName == "void")
                {
                    funcArgs.Add(string.Format("IntPtr {0}", argName));
                    callArgs.Add(string.Format("(IntPtr){0}", argName));
                }
                else
                {
                    funcArgs.Add(string.Format("{0} {1} {2}", argType.IsConstQualified ? "in" : "out", argTypeName, argName));
                    fixedArgs.Add(string.Format("fixed ({0}* _{1} = &{1})", argTypeName, argName));
                    callArgs.Add(string.Format("(IntPtr)_{0}", argName));
                }
            }
            else
            {
                if (!string.IsNullOrEmpty(defaultParam))
                {
                    defaultParam = " = " + defaultParam;
                }

                funcArgs.Add(string.Format("{0} {1}{2}", argTypeName, argName, defaultParam));
                callArgs.Add(string.Format("{0}", argName));
            }
        }

        string funcDeclStr = string.Format("public unsafe {0}{1} {2}", (isStatic || extension) ? "static " : "", retTypeName, methodName);

        if (0 < constraints.Count)
        {
            funcDeclStr += "<";

            for (int i = 0; i < constraints.Count; ++i)
            {
                if (i > 0)
                {
                    funcDeclStr += ", ";
                }
                funcDeclStr += "T" + i;
            }

            funcDeclStr += ">";
        }

        funcDeclStr += "(" + string.Join(", ", funcArgs) + ")";

        code.Append(funcDeclStr);

        if (0 < constraints.Count)
        {
            for (int i = 0; i < constraints.Count; ++i)
            {
                code.Append("where T{0}: unmanaged, Bullet.{1}", i.ToString(), constraints[i]);
            }
        }

        code.BeginScope();

        if (fixedArgs.Count > 0)
        {
            foreach (var arg in fixedArgs)
            {
                code.Append(arg);
            }
            code.BeginScope();
        }

        string call = string.Format("Library.{0}({1})", handle.Spelling.ToString(), string.Join(", ", callArgs));

        if (returnsClassPtr)
        {
            code.Append("return new {0}() {{ Ptr = (IntPtr){1} }};", retTypeName, call);
        }
        else if (returnsPtr)
        {
            code.Append("return (IntPtr){0};", call);
        }
        else if (retTypeName != "void")
        {
            code.Append("return {0};", call);
        }
        else
        {
            code.Append("{0};", call);

            if (GetAnnotations(handle).Contains("DESTRUCTOR"))
            {
                code.Append("self.Ptr = (IntPtr)null;");
            }
        }

        if (fixedArgs.Count > 0)
        {
            code.EndScope();
        }

        code.EndScope();
    }

    void WriteClassEnums(CodeBuilder code, ClassInfo classInfo)
    {
        foreach (var e in classInfo.Enums)
        {
            WriteEnum(code, e);
        }
    }

    void WriteEnum(CodeBuilder code, CXCursor handle)
    {
        string name = handle.Spelling.ToString();

        code.Append("[Flags]");
        code.Append("public enum " + name);
        code.BeginScope();

        handle.VisitChildren((cursor, parent, client_data) =>
        {
            if (cursor.Kind == CXCursorKind.CXCursor_EnumConstantDecl)
            {
                code.Append("{0} = {1},", cursor.Spelling.ToString(), clang.getEnumConstantDeclValue(cursor).ToString());
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, new CXClientData());

        code.EndScope();
    }

    static string GetMethodName(CXCursor handle, ClassInfo classInfo)
    {
        string result = handle.Spelling.ToString();

        if (result.StartsWith(classInfo.MethodPrefix))
        {
            result = result.Substring(classInfo.MethodPrefix.Length);
        }

        Match m = _regexFuncName.Match(result);
        if (m.Success)
        {
            result = m.Groups[1].Value;
        }

        return result;
    }

    static string GetInterfaceName(CXCursor handle)
    {
        string result = "I" + handle.Spelling.ToString();
        return result;
    }

    static string GetFullInterfaceName(CXCursor handle)
    {
        string result = GetInterfaceName(handle);

        if (handle.SemanticParent.kind != CXCursorKind.CXCursor_TranslationUnit)
        {
            result = GetDottedName(handle.SemanticParent) + "." + result;
        }

        return result;
    }

    static string GetDottedName(string s)
    {
        return s.Replace("::", ".");
    }

    static string GetDottedName(CXCursor handle)
    {
        string result = GetFullyQualifiedName(handle);
        return result.Replace("::", ".");
    }

    static string GetFullyQualifiedName(CXCursor handle)
    {
        if (handle.IsInvalid || handle.Kind == CXCursorKind.CXCursor_TranslationUnit)
        {
            return "";
        }

        string result = GetFullyQualifiedName(handle.SemanticParent);
        if (!string.IsNullOrEmpty(result))
        {
            return result + "::" + handle.Spelling;
        }

        return handle.Spelling.ToString();
    }

    static bool IsForwardDeclaration(CXCursor cursor)
    {
        var definition = clang.getCursorDefinition(cursor);

        if (clang.equalCursors(definition, clang.getNullCursor()) != 0)
        {
            return true;
        }

        return clang.equalCursors(cursor, definition) == 0;
    }

    static bool IsStaticMethod(string[] annotations)
    {
        return annotations.Contains("STATIC");
    }

    void CheckTranslationUnitErrors(TranslationUnit translationUnit, string sourceContents)
    {
        if (translationUnit.Handle.NumDiagnostics == 0)
        {
            return;
        }

        var errorDiagnostics = new StringBuilder();
        errorDiagnostics.AppendLine($"The provided {nameof(CXTranslationUnit)} has the following diagnostics which prevent its use:");
        var invalidTranslationUnitHandle = false;

        string[] lines = null;

        for (uint i = 0; i < translationUnit.Handle.NumDiagnostics; ++i)
        {
            using (var diagnostic = translationUnit.Handle.GetDiagnostic(i))
            {
                if ((diagnostic.Severity == CXDiagnosticSeverity.CXDiagnostic_Error) || (diagnostic.Severity == CXDiagnosticSeverity.CXDiagnostic_Fatal))
                {
                    diagnostic.Location.GetFileLocation(out var file, out var line, out var col, out var offset);

                    if (lines == null)
                    {
                        lines = sourceContents.Split('\n');
                    }

                    invalidTranslationUnitHandle = true;
                    errorDiagnostics.Append(' ', 4);
                    errorDiagnostics.AppendLine(diagnostic.Format(CXDiagnosticDisplayOptions.CXDiagnostic_DisplayOption).ToString());
                    errorDiagnostics.AppendLine("------------------------------------------------");

                    if (0 < line)
                    {
                        errorDiagnostics.AppendLine(lines[line - 1]);
                    }

                    errorDiagnostics.AppendLine(lines[line] + " <<<");

                    if (line < lines.Length - 1)
                    {
                        errorDiagnostics.AppendLine(lines[line + 1]);
                    }

                    errorDiagnostics.AppendLine("");
                }
            }
        }

        if (invalidTranslationUnitHandle)
        {
            throw new ArgumentOutOfRangeException(nameof(translationUnit), errorDiagnostics.ToString());
        }
    }

    void VisitTranslationUnit(TranslationUnit translationUnit)
    {
        var translationUnitDecl = translationUnit.TranslationUnitDecl;

        foreach (var decl in translationUnit.TranslationUnitDecl.Decls)
        {
            if (decl.Kind == CXCursorKind.CXCursor_StructDecl || decl.Kind == CXCursorKind.CXCursor_ClassDecl || decl.Kind == CXCursorKind.CXCursor_ClassTemplate)
            {
                VisitClassOrStructDecl(decl.Handle);
            }
        }

        foreach (var decl in translationUnit.TranslationUnitDecl.Decls)
        {
            if (decl.Kind == CXCursorKind.CXCursor_FunctionDecl)
            {
                VisitFunctionDecl(decl.Handle);
            }
        }
    }

    string[] GetAnnotations(CXCursor handle)
    {
        List<string> result = new List<string>();

        handle.VisitChildren((cursor, parent, clientData) =>
        {
            if (cursor.kind == CXCursorKind.CXCursor_AnnotateAttr)
            {
                result.Add(cursor.ToString());
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, clientData: new CXClientData());

        return result.ToArray();
    }

    void VisitFunctionDecl(CXCursor handle)
    {
        var annotations = GetAnnotations(handle);

        bool managed = annotations.Contains("MANAGED");
        bool aligned = annotations.Contains("ALIGNED");

        if (managed || aligned)
        {
            var arg = clang.Cursor_getArgument(handle, 0);
            string managedClassName = GetFullyQualifiedName(arg.Type.Declaration);

            var classInfo = _classMap[managedClassName];
            classInfo.IsManagedStruct = true;
            classInfo.IsAligned = aligned;

            if (classInfo.Export == ExportSetting.Default)
            {
                classInfo.Export = ExportSetting.Export;
            }

            return;
        }

        foreach (var annotation in annotations)
        {
            if (annotation.StartsWith("CLASS:"))
            {
                string className = annotation.Substring("CLASS:".Length);
                AddClassMethod(_classMap[className], handle);
            }
        }
    }

    void AddClassMethod(ClassInfo classInfo, CXCursor handle)
    {
        var resultType = clang.getCursorResultType(handle);

        if (resultType.kind != CXTypeKind.CXType_Void)
        {
            if (resultType.kind == CXTypeKind.CXType_Enum)
            {
                AddEnum(resultType.Declaration);
            }
        }

        int numArgs = clang.Cursor_getNumArguments(handle);

        for (int i = 0; i < numArgs; ++i)
        {
            var arg = clang.Cursor_getArgument(handle, (uint)i);
            if (arg.Type.CanonicalType.kind == CXTypeKind.CXType_Enum)
            {
                AddEnum(arg.Type.CanonicalType.Declaration);
            }
        }

        classInfo.Methods.Add(handle);

        for (var c = classInfo; c != null && c.Export == ExportSetting.Default; c = c.Base)
        {
            c.Export = ExportSetting.Export;
        }
    }

    void VisitClassOrStructDecl(CXCursor handle, ClassInfo parentClass = null)
    {
        if (IsForwardDeclaration(handle))
        {
            return;
        }

        ClassInfo top = new ClassInfo(handle);

        var annotations = GetAnnotations(handle);

        if (annotations.Contains("NOEXPORT"))
        {
            top.Export = ExportSetting.Ignore;
        }

        _allClasses[handle] = top;
        _classMap[top.FullyQualifiedName] = top;

        if (parentClass == null)
        {
            parentClass = _root;
        }

        parentClass.ChildClasses.Add(top);

        handle.VisitChildren((cursor, parent, clientData) =>
        {
            if (cursor.Kind == CXCursorKind.CXCursor_StructDecl || cursor.Kind == CXCursorKind.CXCursor_ClassDecl)
            {
                VisitClassOrStructDecl(cursor, top);
            }
            else if (cursor.kind == CXCursorKind.CXCursor_CXXBaseSpecifier)
            {
                if (cursor.Definition.kind == CXCursorKind.CXCursor_ClassDecl || cursor.Definition.kind == CXCursorKind.CXCursor_StructDecl)
                {
                    if (0 < cursor.Type.NumTemplateArguments)
                    {
                        top.Base = _classMap[GetFullyQualifiedName(cursor.Type.Declaration)];
                    }
                    else
                    {
                        top.Base = _allClasses[cursor.Definition];
                    }
                }
            }
            return CXChildVisitResult.CXChildVisit_Continue;
        }, clientData: new CXClientData());
    }

    void AddEnum(CXCursor handle)
    {
        if (_exportingEnums.Contains(handle))
        {
            return;
        }

        _exportingEnums.Add(handle);

        var semanticParent = handle.SemanticParent;
        if (semanticParent.kind == CXCursorKind.CXCursor_TranslationUnit)
        {
            _root.Enums.Add(handle);
        }
        else
        {
            _allClasses[semanticParent].Enums.Add(handle);
        }
    }

    static void Main(string[] args)
    {
        CodeBuilder code = new CodeBuilder();

        Bullet4UnityPInvokeGenerator pinvokeGenerator = new Bullet4UnityPInvokeGenerator();
        pinvokeGenerator.Run(args, code);

        Console.WriteLine(code.ToString());
    }
}

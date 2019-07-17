using System;
using System.Collections.Generic;
using System.Text;

class CodeBuilder
{
    int _indent;
    List<string> _lines = new List<string>();
    string _indentString = "";

    public void BeginScope()
    {
        Append("{");
        ++_indent;
        UpdateIndentString();
    }

    public void EndScope()
    {
        --_indent;
        UpdateIndentString();
        Append("}");
    }

    void UpdateIndentString()
    {
        _indentString = "";
        for (int i = 0; i < _indent; ++i)
        {
            _indentString += "\t";
        }
    }

    public void Append(CodeBuilder code)
    {
        foreach (var s in code._lines)
        {
            Append(s);
        }
    }

    public void Append(string s)
    {
        _lines.Add(_indentString + s);
    }

    public void Append(string format, params string[] s)
    {
        _lines.Add(_indentString + string.Format(format, s));
    }

    public override string ToString()
    {
        return string.Join("\n", _lines.ToArray());
    }
}

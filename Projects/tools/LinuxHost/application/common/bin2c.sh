# !/bin/bash

lengthSuffix="_Length"
sourceSuffix="_Source"

if [ ! -e $1 ] || [ -z "$2" ] || [ -z "$3" ]; then
    echo "$0 v1.1 - Convert a file to a C file representing its contents as a binary array."
    echo ""
    echo "Usage: $0 <InFile> <VariableNameToUseInC> <OutFileWithoutExtension>"
    echo "       Reads from <InpFile>."
    echo "       Creates <OutFile>.c & <OutFile>.h (overwriting already exists) with 3 variables:"
    echo "            A 'const char *' named <VariableNameToUseInC>$sourceSuffix containing the base name of <InFile>, and"
    echo "            a 'const unsigned char []' named <VariableNameToUseInC>, and"
    echo "            a 'const unsigned int' named <VariableNameToUseInC>$lengthSuffix."
    echo ""
    echo "Exits with 0 if success, otherwise non-zero."
    rc=1
else
    sourceFileName=`basename $1`
    hFile=$3.h
    cFile=$3.c
    imageLength=`ls -l $1 | cut -d ' ' -f 5`
    imageDataRaw=`hexdump -v -e '/1 "0x%02x,"' $1`
    # Create H file
    echo "extern const char *$2$sourceSuffix;" > $hFile
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "ERROR: Unable to create $hFile"
    else
    	echo "extern const unsigned int  $2$lengthSuffix;" >> $hFile
        echo "extern const unsigned char $2[$imageLength];" >> $hFile
        # Now create C file
        echo "#include \"$hFile\"" > $cFile
        rc=$?
        if [ $rc -ne 0 ]; then
            echo "ERROR: Unable to create $cFile"
        else
            echo "const char *$2$sourceSuffix = \"$sourceFileName\";" > $cFile
            echo "const unsigned int  $2$lengthSuffix = $imageLength;" >> $cFile
            echo "const unsigned char $2[$imageLength] = {" >> $cFile
            # Strip final trailing comma, and for readibility, add leading tab, and replace evey 16th comma with a comma, line-break & tab.
            echo "${imageDataRaw::-1}" | sed -e's|^|\t|g' | sed -e's|\(\([^,]*,\)\{15\}[^,]*\),|\1,\n\t|g' >> $cFile
            echo "};" >> $cFile
            echo "Created files $cFile and $hFile with data from $1 containing variables $2 and $2$lengthSuffix"
        fi
    fi
fi
exit $rc


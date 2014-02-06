#!/bin/sh -e

if [ -f .git/hooks/pre-commit.sample -a ! -f .git/hooks/pre-commit ] ; then
        cp -p .git/hooks/pre-commit.sample .git/hooks/pre-commit && \
        chmod +x .git/hooks/pre-commit && \
        echo "Activated pre-commit hook."
fi

autoreconf --install --symlink

libdir() {
        echo $(cd $1/$(gcc -print-multi-os-directory); pwd)
}

args="--prefix=/usr \
--sysconfdir=/etc \
--libdir=$(libdir /usr/lib)"

echo
echo "----------------------------------------------------------------"
echo "Initialized build system. For a common configuration please run:"
echo "----------------------------------------------------------------"
echo
echo "For a system installation:"
echo "./configure CXXFLAGS='-g -O2' CFLAGS='-g -O2' $args"
echo ""
echo "For a user installation:"
echo "./configure CXXFLAGS='-g -O2' CFLAGS='-g -O2' --prefix=$HOME/usr"
echo

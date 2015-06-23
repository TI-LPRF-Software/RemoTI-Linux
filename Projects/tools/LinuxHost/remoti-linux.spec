Name: %{_name}
Summary: Summary test
Version: %{_version}
License: Proprietary
Release: %{_buildnumber}
BuildArch: %{_arch}
Prefix: %{_prefix}

Requires: bootstrap-witbe

AutoReqProv: no

%description
none

%prep

%build

%pre

%preun
systemctl stop %{_name}.service
systemctl disable %{_name}.service
systemctl daemon-reload

%install
mkdir -p $RPM_BUILD_ROOT/usr/

cp -rf %{_installPath}/etc/ $RPM_BUILD_ROOT/
cp -rf %{_installPath}/include/ $RPM_BUILD_ROOT/usr/
cp -rf %{_installPath}/lib/ $RPM_BUILD_ROOT/usr/
cp -rf %{_installPath}/bin/ $RPM_BUILD_ROOT/usr/

%post
systemctl daemon-reload
systemctl start %{_name}.service
systemctl enable %{_name}.service

%postun

%files
%defattr(-,root,root)
/etc/RemoTI
/usr/lib/libRemoTI_client.so
/usr/lib/systemd/system/%{_name}.service
/usr/include/RemoTI
/usr/bin/RemoTI_server

%exclude

%clean

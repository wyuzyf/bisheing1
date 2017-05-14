function varargout = readfromexcel(fileloc,varargin)
% VARARGOUT = READFROMEXCEL(FILELOC,VARARGIN)
%
%             Uses ActiveX commands to read range(s) from an existing Excel
%             spreadsheet.
%
% FILELOC:            Enter a string representing the (absolute or relative) 
%                     location of an Excel file. (Extension may be
%                     omitted, and will be assumed to be .xls.)
%                     Examples: 'c:\brett\my archives\test1.xls'
%                               'test1.xls'
%                               'myarchive'
%
% SHEETNAME:          (Optional): Any occurrence in the variable argument list of
%                     the strings 'sheetname' or 'sheet' prompts the function to change
%                     active sheets to the value in the following variable. That specifier
%                     must be a string argument matching exactly the name of an existing 
%                     sheet in the opened file. If this argument is omitted, the function
%                     defaults to reading from the first sheet in the file.
%
% RANGE SPECIFIER(S): Enter the range(s) to read. The values stored in these
%                     ranges will be returned in consecutive output arguments.
%                     Example: 'B1:B5'
%                              'B1:B1' (or simply 'B1')
%                              'B1:P4'
%                              'B:B' or 'B' (Entire second column)
%                              '2:2' or '2' (Entire second row)
%                              'ALL' (Entire sheet)
%      (Additional ranges: Comma separated ranges in the same form as above;
%                contents of archive will be returned in output arguments
%                2...n)
%
% NOTE:               Specifying range as 'ALL' returns entire used portion of sheet;
%                     Specifying range as 'B:B' or '2:2' returns
%                     appropriate row of UsedRange. (Data are selected in
%                     block form as for 'ALL', then the selected row/column
%                     is returned.
%
% OUTPUT: If specified range is 1 cell, variable returned is of the same
%         class as cell contents. If the range spans more than 1 cell, the variables will be cell arrays.
%
% EXAMPLES:   a = readfromexcel('c:\brett\my archives\test1.xls','C1:C5');
%                 reads from the currently active sheet
%             [a,b] = readfromexcel('c:\brett\my archives\test1.xls','sheet','sheet2','C1:C5','C1:P3');
%                 reads from sheet2
%             [a,b,c] = readfromexcel('myarchive','C3:D5','sheet','mysheet','E4','sheet','sheet2','B3');
%                 reads a from currently active sheet, switches to sheet
%                 'mysheet' to read b, then to sheet 'sheet2' to read c.
%
% Written by Brett Shoelson, Ph.D.
% shoelson@helix.nih.gov
% Update History: 1/04. Version 1.
%                 2/2/04. Now allows multiple specifications of sheet name
%                    (at the suggestion of R. Venkat), and support of
%                    relative paths (thanks to Urs Schwarz). Also, inclusion of the
%                    extension '.xls' is now superfluous.
%                 7/21/04. Implements try/catch structure for reading of
%                    ranges to avoid errors that leave open activex
%                    connections. (Response to Chris Paterson's CSSM
%                    query).
%                 7/29/04. Accomodates reading of entire sheet, or of
%                    entire row/column. (Response to email queries by Kinan Rai
%                    and CSSM query by Xiong.)
%
% SEE ALSO: write2excel

if nargin < 2
	msgstr = sprintf('\nAt a minimum, you must specify three input arguments.\nThe first is a string indicating the location of the excel file,\nand the second is a range to be read.');
	error(msgstr);
end

sheetchanges = [strmatch('sheet',varargin,'exact');strmatch('sheetname',varargin,'exact')];
if ~isempty(sheetchanges)
	[sheetnames{1:length(sheetchanges)}] = deal(varargin{sheetchanges+1});
end

[pathstr,name,ext] = fileparts(fileloc);
if isempty(ext)
	fileloc = [fileloc,'.xls'];
end
if isempty(pathstr)
	fileloc = which(fileloc,'-all');
	if size(fileloc,1) ~= 1
		error('File was either not located, or multiple locations were found. Please reissue readfromexcel command, providing absolute path to the file of interest.');
	end
end

Excel = actxserver('Excel.Application'); 
Excel.Visible = 0; 
w = Excel.Workbooks; 
try
	excelarchive = invoke(w, 'open', fileloc);
catch
	invoke(Excel, 'quit'); 
	release(w); 
	delete(Excel);
	error(sprintf('Sorry...unable to open file %s',fileloc));
end

Sheets = Excel.ActiveWorkBook.Sheets;

archive = Excel.Activesheet; 
initval = get(archive,'Index');

% Read appropriate ranges into output variables
chgcount = 1; argcount = 1;
for ii = 1:nargin-1
	readinfo = [];
	if ismember(ii,sheetchanges)
		try
			sheet = get(Sheets,'Item',sheetnames{chgcount});
			invoke(sheet,'Activate');
			archive = Excel.Activesheet;
			chgcount = chgcount + 1;
			continue
		catch
			invoke(Excel, 'quit'); 
			release(w); 
			delete(Excel);
			error(sprintf('\nUnable to find/open sheet %s.',sheetnames{chgcount}));
		end
	elseif ismember(ii,sheetchanges + 1)
		continue
	end
	
	%Parse range
	rangespec = 0;
	if strcmp(lower(varargin{ii}),'all') %Range of the form 'ALL'
		rangespec = 1;
	else
		tmp = findstr(varargin{ii},':');
		if isempty(tmp) %Range of the form 'A' or '2' or 'A2'
			r1 = varargin{ii};
			r2 = r1;
			if ~any(ismember(r1,num2str([1:9]))) %Range of the form 'A'
				rangespec = 2;
			elseif all(ismember(r1,num2str([1:9]))) %Range of the form '2'
				rangespec = 3;
			else %Range of the form 'A2'
				rangespec = 4;
			end
		else % Range of the form 'A2:B3', '2:2', or 'A:A'
			r1 = varargin{ii}(1:tmp-1);
			if all(ismember(r1,num2str([1:9]))) %Range of the form '2:2'
				r2 = r1;
				rangespec = 5;
			elseif ~any(ismember(r1,num2str([1:9]))) %Range of the form 'A:A'
				r2 = r1;
				rangespec = 6;
			else
				%Range of the form 'A1:B2'
				r2 = varargin{ii}(tmp+1:end);
				rangespec = 7;
			end			
		end
	end
	
	try
		switch rangespec
			case 1
				readinfo = get(archive,'UsedRange');
			case {2,6}
				readinfo = get(archive,'UsedRange');
				[r,c] = an2nn(r1);
				r1 = nn2an(readinfo.row,c);
				[m,n] = size(readinfo.value);
				r2 = nn2an(readinfo.row+m,c);
				readinfo = get(archive, 'Range', r1, r2);
			case {3,5}
				readinfo = get(archive,'UsedRange');
				[m,n] = size(readinfo.value);
				r2 = nn2an(r1,readinfo.row+n);
				r1 = nn2an(r1,readinfo.column);
				readinfo = get(archive, 'Range', r1, r2);
			case {4,7}
				readinfo = get(archive, 'Range', r1, r2);
			otherwise
				readinfo.value = {};
				fprintf('Error parsing input argument %d.',ii+1);
		end
	catch
		fprintf('Error reading range specified by input argument %d.',ii+1);
		invoke(Excel, 'quit'); 
		release(excelarchive); 
		release(w); 
		delete(Excel);
		return
	end
	varargout{argcount} = readinfo.value;
	argcount = argcount + 1;
end
% Reset to initial active sheet
sheet = get(Sheets,'Item',initval);
invoke(sheet,'Activate');

try
	release(readinfo); 
end
%invoke(excelarchive,'close'); %This closes without saving, so changing the active sheet is temporary
invoke(excelarchive,'save'); %Note: Instead of invoke(excelarchive,'close'), I use the save option after
%                             switching back to the initially active sheet. This stops Excel from showing
%                             "previously saved versions" when the file is
%                             next opened.
invoke(Excel, 'quit'); 
release(excelarchive); 
release(w); 
delete(Excel);
return


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SUBFUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cr = nn2an(r, c)
% convert number, number format to alpha, number format
%t = [floor(c/27) + 64 floor((c - 1)/26) - 2 + rem(c - 1, 26) + 65]; 
t = [floor((c - 1)/26) + 64 rem(c - 1, 26) + 65]; 
if(t(1)<65), t(1) = []; end
cr = [char(t) num2str(r)]; 

function [r, c] = an2nn(cr)
% convert alpha, number format to number, number format
t = find(isletter(cr)); 
t2 = abs(upper(cr(t))) - 64; 
if(length(t2) == 2), t2(1) = t2(1) * 26; end
c = sum(t2); r = str2num(cr(max(t) + 1:length(cr))); 



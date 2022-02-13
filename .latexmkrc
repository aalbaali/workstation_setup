$ENV{'TZ'}='Canada/Eastern';

$clean_ext='aux fdb_latexmk fls log';

$compiling_cmd="echo \"\033[96;1mCompiling \'%T\' renamed as \'%R\' to \'%Z\' directory\033[0m\"";
$success_cmd="echo \"\033[92;1mSuccessfully compiled \'%R\' to \'%Z\'\033[0m\"";

@default_files=("src/*.tex");

$jobname="%A";

$pdf_mode=1;

$out_dir="build"
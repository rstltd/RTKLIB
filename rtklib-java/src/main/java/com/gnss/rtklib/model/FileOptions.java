package com.gnss.rtklib.model;

/**
 * File options, matching C RTKLIB's filopt_t.
 */
public class FileOptions {
    public String satantp = "";   // file-satantfile
    public String rcvantp = "";   // file-rcvantfile
    public String stapos  = "";   // file-staposfile
    public String geoid   = "";   // file-geoidfile
    public String iono    = "";   // file-ionofile
    public String dcb     = "";   // file-dcbfile
    public String eop     = "";   // file-eopfile
    public String blq     = "";   // file-blqfile
    public String tempdir = "";   // file-tempdir
    public String geexe   = "";   // file-geexefile
    public String solstat = "";   // file-solstatfile
    public String trace   = "";   // file-tracefile
}

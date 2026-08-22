-- -----------------------------------------------------------------------------
-- tb_pkg
--
-- Shared testbench machinery: reading the generated vector files, comparing
-- against them and ending the run with an exit code the build can act on.
--
-- Every testbench in this repository is self-checking.  None of them produce a
-- waveform for a person to look at and none of them pass by not crashing: each
-- one counts its comparisons, reports the count and fails the simulation if any
-- comparison did not hold.  A testbench that reports zero comparisons fails too,
-- because a vector file that failed to open would otherwise look like a pass.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library std;
  use std.textio.all;
  use std.env.all;

package tb_pkg is

  -- Exact comparison.  The expected values come from a bit-accurate model, so
  -- anything other than equality is a defect in one of the two.

  procedure check_int (
    name : in    string;
    got  : in    integer;
    exp  : in    integer;
    errs : inout natural
  );

  -- Comparison against a derived bound, used only where the reference is real
  -- arithmetic rather than the model.

  procedure check_near (
    name : in    string;
    got  : in    integer;
    exp  : in    integer;
    tol  : in    integer;
    errs : inout natural
  );

  -- std_logic to 0 or 1, so that flags can be compared with vector columns.

  function to_int (s : std_logic) return integer;

  procedure check_true (
    name : in    string;
    cond : in    boolean;
    errs : inout natural
  );

  -- Next line of a vector file that is neither blank nor a comment.

  procedure next_data_line (
    file f : text;
    l      : inout line;
    ok     : out   boolean
  );

  -- Report and end the simulation.  Fails on any error and on no checks at all.

  procedure finish_report (
    name   : in string;
    errs   : in natural;
    checks : in natural
  );

end package tb_pkg;

package body tb_pkg is

  procedure check_int (
    name : in    string;
    got  : in    integer;
    exp  : in    integer;
    errs : inout natural
  ) is
  begin

    if got /= exp then
      errs := errs + 1;
      if errs <= 20 then
        report name & ": got " & integer'image(got)
               & ", expected " & integer'image(exp)
          severity error;
      end if;
    end if;

  end procedure check_int;

  procedure check_near (
    name : in    string;
    got  : in    integer;
    exp  : in    integer;
    tol  : in    integer;
    errs : inout natural
  ) is
  begin

    if abs (got - exp) > tol then
      errs := errs + 1;
      if errs <= 20 then
        report name & ": got " & integer'image(got)
               & ", expected " & integer'image(exp)
               & " within " & integer'image(tol)
          severity error;
      end if;
    end if;

  end procedure check_near;

  function to_int (s : std_logic) return integer is
  begin

    if s = '1' then
      return 1;
    else
      return 0;
    end if;

  end function to_int;

  procedure check_true (
    name : in    string;
    cond : in    boolean;
    errs : inout natural
  ) is
  begin

    if not cond then
      errs := errs + 1;
      if errs <= 20 then
        report name & ": condition did not hold"
          severity error;
      end if;
    end if;

  end procedure check_true;

  procedure next_data_line (
    file f : text;
    l      : inout line;
    ok     : out   boolean
  ) is

    variable ln : line;

  begin

    ok := false;

    loop

      if endfile(f) then
        return;
      end if;

      readline(f, ln);

      if ln.all'length > 0 then
        if ln.all(ln.all'left) /= '#' then
          l  := ln;
          ok := true;
          return;
        end if;
      end if;

      deallocate(ln);

    end loop;

  end procedure next_data_line;

  procedure finish_report (
    name   : in string;
    errs   : in natural;
    checks : in natural
  ) is
  begin

    report name & ": " & integer'image(checks) & " checks, "
           & integer'image(errs) & " errors"
      severity note;
    assert checks > 0
      report name & ": no checks ran, the vector file was probably not found"
      severity failure;
    assert errs = 0
      report name & ": FAILED"
      severity failure;
    finish;

  end procedure finish_report;

end package body tb_pkg;

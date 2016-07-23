using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace UIWindow {
public partial class AnnotatorWindow : Form {
  UIWindow parent;
  string annotationType;
  string defaultExt = "json";
  string defaultFilter = "JSON files (*.json)|*.json|All files (*.*)|*.*";
  string defaultFilename = "";
  string defaultDirectory = Environment.CurrentDirectory;

  int iId = 0;
  int iLabel = 1;
  int iInfo = 2;
  int iObjectId = 3;

  readonly object stateLock = new object();
  bool ignoreCellChanges = false;

  public AnnotatorWindow(UIWindow _parent, string _annotationType)
  {
    InitializeComponent();
    // Set double buffered for smoother UI update
    annotationsTable.DoubleBuffered(true);
    parent = _parent;
    annotationType = _annotationType;
  }

  public void setFilename(string filename) {
    lock (stateLock) {
      if (defaultFilename != filename) {
        defaultFilename = filename;
        clearAnnotations();
      }
    }
  }

  public void setDirectory(string directory) {
    lock (stateLock) {
      defaultDirectory = directory;
    }
  }

  public void process(string action, string id, string label, string info, string objectId)
  {
    lock (stateLock) { 
      // Adding annotation
      if (action == "add") {
        annotationsTable.Rows.Add(id, label, info, objectId);
      } else if (action == "select") {
        int rowIndex = findRowIndex(id);
        if (rowIndex >= 0 && rowIndex < annotationsTable.Rows.Count) {
          if (info != null) {
            annotationsTable.Rows[rowIndex].Cells[iInfo].Value = info;
          }
          annotationsTable.CurrentCell = annotationsTable.Rows[rowIndex].Cells[iLabel];
        }
      } else if (action == "update") {
        ignoreCellChanges = true;
        int rowIndex = findRowIndex(id);
        if (rowIndex >= 0 && rowIndex < annotationsTable.Rows.Count) {
          if (label != null) {
            annotationsTable.Rows[rowIndex].Cells[iLabel].Value = label;
          }
          if (info != null) {
            annotationsTable.Rows[rowIndex].Cells[iInfo].Value = info;
          }
          if (objectId != null) {
            annotationsTable.Rows[rowIndex].Cells[iObjectId].Value = objectId;
          }
        }
        ignoreCellChanges = false;
      } else if (action == "clear") {
        clearAnnotations();
      }
    }
  }

  private void AnnotatorWindow_Load(object sender, EventArgs e) {
  }

  // Save annotations to file
  private void SaveButton_Click(object sender, EventArgs e) {
    lock (stateLock) {
      string path;
      SaveFileDialog file = new SaveFileDialog();
      file.FileName = defaultFilename;
      file.DefaultExt = defaultExt;
      file.InitialDirectory = defaultDirectory;
      file.RestoreDirectory = true;

      if (file.ShowDialog() == DialogResult.OK) {
        path = file.FileName;
        parent.SendMessage("D3D:" + annotationType + ":save " + path);
      }
    }
  }

  // Load annotations from file
  private void LoadButton_Click(object sender, EventArgs e)
  {
    lock (stateLock) {
      string path;
      OpenFileDialog file = new OpenFileDialog();
      file.FileName = defaultFilename;
      file.Filter = defaultFilter;
      file.InitialDirectory = defaultDirectory;
      file.RestoreDirectory = true;

      if (file.ShowDialog() == DialogResult.OK) {
        path = file.FileName;
        parent.SendMessage("D3D:" + annotationType + ":load " + path);
      }
    }
  }

  // Delete annotation
  private void annotationsTable_UserDeletingRow(object sender, DataGridViewRowCancelEventArgs e)
  {
    lock (stateLock) {
      string selId = getId(e.Row.Index);
      parent.SendMessage("D3D:" + annotationType + ":delete " + selId);
    }
  }

  // Annotation changed
  private void annotationsTable_CellValueChanged(object sender, DataGridViewCellEventArgs e)
  {
    if (ignoreCellChanges) return;
    lock (stateLock) {
      if (e.RowIndex >= 0 && e.RowIndex < annotationsTable.Rows.Count) {
        if (e.ColumnIndex >= 0 && e.ColumnIndex < annotationsTable.Columns.Count) {
          // Valid column - For now we only care about changes to the "label"
          if (e.ColumnIndex == iLabel) {
            int rowIndex = annotationsTable.SelectedCells[0].RowIndex;
            string selId = getId(rowIndex);
            string label = getLabel(rowIndex);
            parent.SendMessage("D3D:" + annotationType + ":relabel " + selId + " label " + label);
          } else if (e.ColumnIndex == iObjectId) {
            int rowIndex = annotationsTable.SelectedCells[0].RowIndex;
            string selId = getId(rowIndex);
            string objectId = getObjectId(rowIndex);
            parent.SendMessage("D3D:" + annotationType + ":relabel " + selId + " objectId " + objectId);
          }
        }
      }
    }
  }

  // Annotation selected
  private void annotationsTable_SelectionChanged(object sender, EventArgs e)
  {
    lock (stateLock) {
      if (annotationsTable.SelectedRows.Count > 0) {
        string selId = getId(annotationsTable.SelectedCells[0].RowIndex);
        parent.SendMessage("D3D:" + annotationType + ":select " + selId);
      }
    }
  }

  private void annotationsTable_CurrentCellChanged(object sender, EventArgs e)
  {
    lock (stateLock) {
      if (annotationsTable.SelectedCells.Count > 0) {
        string selId = getId(annotationsTable.SelectedCells[0].RowIndex);
        parent.SendMessage("D3D:" + annotationType + ":select " + selId);
      }
    }
  }

  // Clear annotations
  private void clearButton_Click(object sender, EventArgs e)
  {
      lock (stateLock)
      {
          clearAnnotations();
          parent.SendMessage("D3D:" + annotationType + ":clear");
      }
  }

  private void AnnotatorWindow_FormClosing(object sender, FormClosingEventArgs e)
  {
      e.Cancel = true;
      this.Hide();
  }

  // Helper functions
  private int findRowIndex(string id)
  {
    for (int idx = 0; idx < annotationsTable.RowCount; ++idx) {
      if (id == getId(idx)) return idx;
    }
    return -1;
  }

  private string getId(int rowIndex)
  {
    return annotationsTable.Rows[rowIndex].Cells[iId].Value.ToString();
  }

  private string getLabel(int rowIndex)
  {
    return annotationsTable.Rows[rowIndex].Cells[iLabel].Value.ToString();
  }

  private string getObjectId(int rowIndex)
  {
    return annotationsTable.Rows[rowIndex].Cells[iObjectId].Value.ToString();
  }

  private void clearAnnotations()
  {
    annotationsTable.Rows.Clear();
    annotationsTable.Refresh();
  }

}
}

// From http://www.dreamincode.net/forums/topic/188953-c%23-datagridview-doublebuffered-property/
//Put this class at the end of the main class or you will have problems.
public static class ExtensionMethods
{
    public static void DoubleBuffered(this DataGridView dgv, bool setting)
    {
        Type dgvType = dgv.GetType();
        PropertyInfo pi = dgvType.GetProperty("DoubleBuffered", BindingFlags.Instance | BindingFlags.NonPublic);
        pi.SetValue(dgv, setting, null);
    }
}


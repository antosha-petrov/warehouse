using System.Collections.ObjectModel;
using WarehouseApp.Services;

namespace WarehouseApp.ViewModel
{
    public class HistoryPageViewModel
    {
        public ObservableCollection<HistoryCardViewModel> Items { get; }

        public HistoryPageViewModel()
        {
            Items = new ObservableCollection<HistoryCardViewModel>(
                AppState.Instance.Goods
                .Select(goods => new HistoryCardViewModel(goods))
            );
        }

        public void RefreshItems()
        {
            Items.Clear();

            var updatedItems = AppState.Instance.Goods
                .Select(goods => new HistoryCardViewModel(goods));

            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }
    }
}
